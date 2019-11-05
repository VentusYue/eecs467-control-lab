include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O3

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	-O3
	 
LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_LCM) \
	  $(LDFLAGS_MAPPING) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_PLANNING) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

BOTGUIDIR = botgui
BOTGUIOBJ = botgui.o botgui_main.o

PUBLISH_COMMANDSOBJ = publish_commands.o

UTILSDIR = utils
UTILSOBJ = drawing_functions.o vx_gtk_window_base.o vx_utils.o

VPATH = $(BOTGUIDIR) $(UTILSDIR)

BIN_BOTGUI = $(BIN_PATH)/botgui
BIN_PUBLISH_COMMANDS = $(BIN_PATH)/publish_commands

ALL = $(BIN_CHALLENGE_GUI) $(BIN_BOTGUI) $(BIN_PUBLISH_COMMANDS)

all: $(ALL)
	
$(BIN_BOTGUI): $(BOTGUIOBJ) $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_CHALLENGE_GUI): $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_PUBLISH_COMMANDS): $(PUBLISH_COMMANDSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a $(BOTGUIDIR)/*.o $(UTILSDIR)/*.o
	@rm -f $(ALL)
