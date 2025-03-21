cmake_minimum_required(VERSION 3.5)
project(minipro)

set(CMAKE_CXX_STANDARD 17)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Log4cxx REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(GLIB REQUIRED glib-2.0)

include_directories(include ${GLIB_INCLUDE_DIRS})

add_library(bluez STATIC
  lib/bluez/att.c
  lib/bluez/bluetooth.c
  lib/bluez/crypto.c
  lib/bluez/gatt-client.c
  lib/bluez/gatt-db.c
  lib/bluez/gatt-helpers.c
  lib/bluez/hci.c
  lib/bluez/io-mainloop.c
  lib/bluez/mainloop.c
  lib/bluez/queue.c
  lib/bluez/timeout-glib.c
  lib/bluez/util.c
  lib/bluez/uuid.c
)

add_library(minipro STATIC
  src/minipro/minipro.cpp
  src/minipro/packet.cpp
  src/minipro/drive.cpp
  src/minipro/enter_remote_control_mode.cpp
  src/minipro/exit_remote_control_mode.cpp
)
target_include_directories(minipro PUBLIC lib/bluez)

add_library(bluetooth STATIC
  src/bluetooth/le_client.cpp
  src/bluetooth/l2_cap_socket.cpp
  src/bluetooth/utils.cpp
)
target_include_directories(bluetooth PUBLIC lib/bluez)

add_library(util STATIC
  src/util/xbox360_controller.cpp
  src/util/joystick.cpp
  src/util/loop_rate.cpp
)

add_executable(gattclient ${BLUEZ_SRC} lib/bluez/btgattclient.c)
target_link_libraries(gattclient bluez ${GLIB_LDFLAGS})
target_include_directories(gattclient PUBLIC lib/bluez)

add_executable(t_minipro ${BLUEZ_SRC} test/minipro/t_minipro.cpp )
target_link_libraries(t_minipro minipro bluetooth util bluez ${GLIB_LDFLAGS} pthread)
target_include_directories(t_minipro PUBLIC lib/bluez)

add_executable(t_joystick test/joystick/t_joystick.cpp)
target_link_libraries(t_joystick util pthread)

