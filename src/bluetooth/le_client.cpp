// Copyright (c) 2020 Michael Jeronimo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, softwars
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bluetooth/le_client.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "bluez.h"
#include "bluetooth/l2_cap_socket.hpp"
#include "bluetooth/utils.hpp"
#include "minipro/minipro.hpp"
#include "util/joystick.hpp"

using namespace std::chrono_literals;
using namespace jeronibot::util;

namespace bluetooth
{

LEClient::LEClient(const std::string & device_address, uint8_t dst_type, int sec, uint16_t mtu)
{
  bdaddr_t dst_addr;
  str2ba(device_address.c_str(), &dst_addr);

  bdaddr_t src_addr;
  bdaddr_t bdaddr_any = {{0, 0, 0, 0, 0, 0}};
  bacpy(&src_addr, &bdaddr_any);

  mainloop_init();

  l2_cap_socket_ = std::make_unique<L2CapSocket>(&src_addr, &dst_addr, dst_type, sec);

  fd_ = l2_cap_socket_->get_handle();
  if (fd_ < 0) {
    throw std::runtime_error("LEClient: Failed to connect to Bluetooth device");
  }

  att_ = bt_att_new(fd_, false);
  if (!att_) {
    bt_att_unref(att_);
    fprintf(stderr, "Failed to initialize ATT transport layer\n");
    return;
  }

  if (!bt_att_set_close_on_unref(att_, true)) {
    bt_att_unref(att_);
    fprintf(stderr, "Failed to set up ATT transport layer\n");
    return;
  }

  if (!bt_att_register_disconnect(att_, LEClient::att_disconnect_cb, nullptr, nullptr)) {
    bt_att_unref(att_);
    fprintf(stderr, "Failed to set ATT disconnect handler\n");
    return;
  }

  // class bluetooth GattClient
  db_ = gatt_db_new();
  if (!db_) {
    bt_att_unref(att_);
    fprintf(stderr, "Failed to create GATT database\n");
    return;
  }

  gatt_ = bt_gatt_client_new(db_, att_, mtu);
  if (!gatt_) {
    gatt_db_unref(db_);
    bt_att_unref(att_);
    fprintf(stderr, "Failed to create GATT client\n");
    return;
  }

  gatt_db_register(db_, service_added_cb, service_removed_cb, nullptr, nullptr);

  bt_gatt_client_set_ready_handler(gatt_, ready_cb, this, nullptr);
  bt_gatt_client_set_service_changed(gatt_, service_changed_cb, this, nullptr);

  // bt_gatt_client already holds a reference
  gatt_db_unref(db_);

  input_thread_ = std::make_unique<std::thread>(std::bind(&LEClient::process_input, this));

  // Wait for client to be ready
  std::unique_lock<std::mutex> lk(mutex_);
  if (cv_.wait_for(lk, 5s, [this] {return ready_;})) {
    printf("LEClient: Ready\n");
  } else {
    throw std::runtime_error("LEClient: Did NOT initialize OK");
  }
}

LEClient::~LEClient()
{
  mainloop_quit();
  bt_gatt_client_unref(gatt_);
  bt_att_unref(att_);
  input_thread_->join();
}

void
LEClient::att_disconnect_cb(int err, void * /*user_data*/)
{
  printf("Device disconnected: %s\n", strerror(err));
  mainloop_quit();
}

void
LEClient::service_added_cb(struct gatt_db_attribute * attr, void * user_data)
{
}

void
LEClient::service_removed_cb(struct gatt_db_attribute * attr, void * user_data)
{
}

void
LEClient::print_uuid(const bt_uuid_t * uuid)
{
  char uuid_str[MAX_LEN_UUID_STR];
  bt_uuid_t uuid128;

  bt_uuid_to_uuid128(uuid, &uuid128);
  bt_uuid_to_string(&uuid128, uuid_str, sizeof(uuid_str));

  printf("%s\n", uuid_str);
}

void
LEClient::print_included_data(struct gatt_db_attribute * attr, void * user_data)
{
  LEClient * This = (LEClient *) user_data;

  uint16_t handle;
  uint16_t start;
  uint16_t end;

  if (!gatt_db_attribute_get_incl_data(attr, &handle, &start, &end)) {
    return;
  }

  struct gatt_db_attribute * service = gatt_db_get_attribute(This->db_, start);
  if (!service) {
    return;
  }

  bt_uuid_t uuid;
  gatt_db_attribute_get_service_uuid(service, &uuid);

  printf("\t  include - handle: 0x%04x, - start: 0x%04x, end: 0x%04x, uuid: ", handle, start, end);
  LEClient::print_uuid(&uuid);
}

void
LEClient::print_descriptor(struct gatt_db_attribute * attr, void * user_data)
{
  printf("\t\t  descr - handle: 0x%04x, uuid: ", gatt_db_attribute_get_handle(attr));
  LEClient::print_uuid(gatt_db_attribute_get_type(attr));
}

void
LEClient::print_characteristic(struct gatt_db_attribute * attr, void * user_data)
{
  uint16_t handle;
  uint16_t value_handle;
  uint8_t properties;
  bt_uuid_t uuid;

  if (!gatt_db_attribute_get_char_data(attr, &handle, &value_handle, &properties, &uuid))
  {
    return;
  }

  printf("\t  charac - start: 0x%04x, value: 0x%04x, " "props: 0x%02x, uuid: ", handle, value_handle, properties);
  LEClient::print_uuid(&uuid);

  gatt_db_service_foreach_desc(attr, print_descriptor, nullptr);
}

void
LEClient::print_service(struct gatt_db_attribute * attr, void * user_data)
{
  LEClient *This = (LEClient *) user_data;

  uint16_t start;
  uint16_t end;
  bool primary;
  bt_uuid_t uuid;

  if (!gatt_db_attribute_get_service_data(attr, &start, &end, &primary, &uuid)) {
    return;
  }

  printf("Service - start: 0x%04x, end: 0x%04x, type: %s, uuid: ", start, end, primary ? "primary" : "secondary");
  LEClient::print_uuid(&uuid);

  gatt_db_service_foreach_incl(attr, print_included_data, This);
  gatt_db_service_foreach_char(attr, print_characteristic, nullptr);

  printf("\n");
}

void
LEClient::ready_cb(bool success, uint8_t att_ecode, void * user_data)
{
  LEClient * This = (LEClient *) user_data;

// TODO: sucess == false on disconnect?

  if (!success) {
    printf("GATT discovery procedures failed - error code: 0x%02x\n", att_ecode);
    return;
  }

  {
    std::lock_guard<std::mutex> lk(This->mutex_);
    This->ready_ = true;
  }

  This->cv_.notify_all();
  This->ready_ = true;
}

void
LEClient::service_changed_cb(uint16_t start_handle, uint16_t end_handle, void * user_data)
{
  LEClient * This = (LEClient *) user_data;

  printf("Service Changed handled - start: 0x%04x end: 0x%04x\n", start_handle, end_handle);
  gatt_db_foreach_service_in_range(This->db_, nullptr, print_service, This, start_handle, end_handle);
}

void
LEClient::read_multiple_cb(
  bool success, uint8_t att_ecode, const uint8_t * value, uint16_t length, void * /*user_data*/)
{
  if (!success) {
    printf("\nRead multiple request failed: 0x%02x\n", att_ecode);
    return;
  }

  printf("\nRead multiple value (%u bytes):", length);
  for (int i = 0; i < length; i++) {
    printf("%02x ", value[i]);
  }
  printf("\n");
}

void
LEClient::read_multiple(uint16_t * handles, uint8_t num_handles)
{
  if (!bt_gatt_client_read_multiple(gatt_, handles, num_handles, read_multiple_cb, nullptr, nullptr)) {
    printf("Failed to initiate read multiple procedure\n");
  }
}

void
LEClient::read_cb(bool success, uint8_t att_ecode, const uint8_t * value, uint16_t length, void * /*user_data*/)
{
  if (!success) {
    printf("Read request failed: %s (0x%02x)\n", bluetooth::utils::to_string(att_ecode), att_ecode);
    return;
  }

  printf("\nRead value");
  if (length == 0) {
    printf(": 0 bytes\n");
    return;
  }

  printf(" (%u bytes): ", length);
  for (int i = 0; i < length; i++) {
    printf("%02x ", value[i]);
  }

  printf("\n");
}

void
LEClient::read_value(uint16_t handle)
{
  if (!bt_gatt_client_read_value(gatt_, handle, read_cb, nullptr, nullptr)) {
    printf("Failed to initiate read value\n");
  }
}

void
LEClient::read_long_value(uint16_t handle, uint16_t offset)
{
  if (!bt_gatt_client_read_long_value(gatt_, handle, offset, read_cb, nullptr, nullptr)) {
    printf("Failed to initiate read long value\n");
  }
}

void
LEClient::write_long_cb(bool success, bool reliable_error, uint8_t att_ecode, void * user_data)
{
  std::promise<int> * promise = (std::promise<int> *) user_data;
  if (success) {
    promise->set_value(0);
  } else if (reliable_error) {
    printf("Reliable write not verified\n");
  } else {
    promise->set_value(att_ecode);
  }
}

void
LEClient::write_long_value(bool reliable_writes, uint16_t handle, uint16_t offset, uint8_t * value, int length)
{
  std::promise<int> promise;
  if (!bt_gatt_client_write_long_value(gatt_, reliable_writes, handle,
      offset, value, length, write_long_cb, (void *) &promise, nullptr))
  {
    printf("Failed to initiate bt_gatt_client_write_long_value\n");
  }

  std::future<int> future = promise.get_future();
  int rc = future.get();
  if (rc != 0) {
    printf("bt_gatt_client_write_long_value failed: %s (0x%02x)\n", bluetooth::utils::to_string(rc), rc);
  }
}

void
LEClient::write_prepare(unsigned int id, uint16_t handle, uint16_t offset, uint8_t * value, unsigned int length)
{
  if (reliable_session_id_ != id) {
    printf("Session id != Ongoing session id (%u!=%u)\n", id, reliable_session_id_);
    return;
  }

  reliable_session_id_ = bt_gatt_client_prepare_write(gatt_, id, handle, offset, value, length,
      // write_long_cb, nullptr, nullptr);
      nullptr, nullptr, nullptr);

  if (!reliable_session_id_) {
    printf("Failed to proceed prepare write\n");
  }

  printf("Prepare write success.\nSession id: %d to be used on next write\n", reliable_session_id_);
}

void
LEClient::write_cb(bool success, uint8_t att_ecode, void * user_data)
{
  std::promise<int> * promise = (std::promise<int> *) user_data;
  promise->set_value(success? 0 : att_ecode);
}

void
LEClient::write_execute(unsigned int session_id, bool execute)
{
  if (execute) {
    std::promise<int> promise;
    if (!bt_gatt_client_write_execute(gatt_, session_id, write_cb, (void *) &promise, nullptr)) {
      printf("Failed to proceed write execute\n");
    }

    std::future<int> future = promise.get_future();
    int rc = future.get();
    if (rc != 0) {
      printf("Write failed: %s (0x%02x)\n", bluetooth::utils::to_string(rc), rc);
    }
  } else {
    bt_gatt_client_cancel(gatt_, session_id);
  }

  reliable_session_id_ = 0;
}

void
LEClient::notify_cb(
  uint16_t value_handle, const uint8_t * value,
  uint16_t length, void * user_data)
{
  printf("Handle Value Not/Ind: 0x%04x - ", value_handle);

  if (length == 0) {
    printf("(0 bytes)\n");
    return;
  }

  printf("(%u bytes): ", length);

  for (int i = 0; i < length; i++) {
    printf("%02x ", value[i]);
  }

  printf("\n");
}

void
LEClient::register_notify_cb(uint16_t att_ecode, void * /*user_data*/)
{
  if (att_ecode) {
    printf("Failed to register notify handler - error code: 0x%02x\n", att_ecode);
    return;
  }

  printf("Registered notify handler!");
}

// TODO(mjeronimo): return unsigned int?
void
LEClient::register_notify(uint16_t value_handle)
{
  unsigned int id = bt_gatt_client_register_notify(
    gatt_, value_handle, register_notify_cb, notify_cb, nullptr, nullptr);

  if (!id) {
    printf("Failed to register notify handler\n");
    return;
  }
}

void
LEClient::unregister_notify(unsigned int id)
{
  if (!bt_gatt_client_unregister_notify(gatt_, id)) {
    printf("Failed to unregister notify handler with id: %u\n", id);
  }
}

void
LEClient::set_security(int level)
{
  if (level < 1 || level > 3) {
    printf("Invalid level: %d\n", level);
    return;
  }

  if (!bt_gatt_client_set_security(gatt_, level)) {
    printf("Could not set security level\n");
  }
}

int
LEClient::get_security()
{
  return bt_gatt_client_get_security(gatt_);
}

bool
LEClient::local_counter(uint32_t * sign_cnt, void * /*user_data*/)
{
  static uint32_t cnt = 0;
  *sign_cnt = cnt++;
  return true;
}

void
LEClient::set_sign_key(uint8_t key[16])
{
  bt_att_set_local_key(att_, key, local_counter, this);
}

void
LEClient::write_value(uint16_t handle, uint8_t * value, int length, bool without_response, bool signed_write)
{
  if (without_response) {
    if (!bt_gatt_client_write_without_response(gatt_, handle, signed_write, value, length)) {
      printf("Failed to initiate write-without-response procedure\n");
    }
  } else {
    std::promise<int> promise;
    if (!bt_gatt_client_write_value(gatt_, handle, value, length, write_cb, (void *) &promise, nullptr)) {
      printf("Failed to initiate write procedure\n");
    }

    std::future<int> future = promise.get_future();
    int rc = future.get();
    if (rc != 0) {
      printf("write_value failed\n");
    }
  }
}

void
LEClient::process_input()
{
  mainloop_run();
}

}  // namespace bluetooth
