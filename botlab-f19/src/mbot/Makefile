include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
CXXFLAGS = $(CXXFLAGS_STD) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_RPLIDAR)
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_DRIVE_SQUARE = $(BIN_PATH)/drive_square
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/rplidar_driver
BIN_SWEEP_DRIVER = $(BIN_PATH)/sweep_driver
BIN_MOTION_CONTROLLER = $(BIN_PATH)/motion_controller
BIN_TIMESYNC = $(BIN_PATH)/timesync
BIN_WIFI_READER = $(BIN_PATH)/wifi_reader
BIN_OLED_EXAMPLE = $(BIN_PATH)/oled_example
BIN_TASK_4 = $(BIN_PATH)/task_4
BIN_WALL_FOLLOWER = $(BIN_PATH)/wall_follower

ALL = $(BIN_RPLIDAR_DRIVER) $(BIN_WIFI_READER) $(BIN_TIMESYNC) $(BIN_DRIVE_SQUARE) $(BIN_MOTION_CONTROLLER) $(BIN_MC_SMOOTH) $(BIN_OLED_EXAMPLE) $(BIN_TASK_4) $(BIN_WALL_FOLLOWER)

all: $(ALL)

$(BIN_DRIVE_SQUARE): drive_square.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MOTION_CONTROLLER): motion_controller.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TIMESYNC): timesync.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_WIFI_READER): wifi_reader.o $(LIBDEPS)
	@echo "    $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_OLED_EXAMPLE): oled_example.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TASK_4): task_4.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_WALL_FOLLOWER): wall_follower.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)
	
clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
