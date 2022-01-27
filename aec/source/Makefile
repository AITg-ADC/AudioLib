MODULE_NAME = libnvtaudlib_aec
# DIRs
OUTPUT_DIR = $(LIBRARY_DIR)/output
# INCs
LIB_INC_PATH = $(LIBRARY_DIR)/include/nvtaudlib_aec
# INC FLAGs
EXTRA_INCLUDE += -I$(LIB_INC_PATH) -I../include
.PHONY: all clean
###############################################################################
# Linux Makefile                                                              #
###############################################################################
ifeq ($(NVT_PRJCFG_CFG),Linux)

############################################################
#### Exactly opt
############################
OPTFLAG += -D_speex_echo_cancellation_OPT
OPTFLAG += -D_filter_dc_notch16_OPT
OPTFLAG += -D_ONLY_FOR_1024_SAMPLES_
OPTFLAG += -D_mdf_adjust_prop_OPT
OPTFLAG += -D_weighted_spectral_mul_conj_OPT
OPTFLAG += -D_compute_gain_floor_OPT
OPTFLAG += -D_compute_gain_floor_NS_OPT
OPTFLAG += -D_Bark_scale_OPT

#### Saturate protection
OPTFLAG += -D_ADD_SUB_SATURATE

#############################################################
# Set flag
#############################################################
DEFINE_FLAGS += -D_ARMV7_
DEFINE_FLAGS += -DHAVE_CONFIG_H 
DEFINE_FLAGS += -DFIXED_POINT
DEFINE_FLAGS += -DADD_NLP

#--------- ENVIRONMENT SETTING --------------------
WARNING	= -Wall -Wundef -Wsign-compare -Wno-missing-braces -Wstrict-prototypes -Wno-sign-compare
COMPILE_OPTS	= -I. -O2 -fPIC -ffunction-sections -fdata-sections -D__LINUX_USER__
C_CFLAGS	= $(PLATFORM_CFLAGS) $(COMPILE_OPTS) $(WARNING) $(EXTRA_INCLUDE)
C_CFLAGS += $(OPTFLAG) 
C_CFLAGS += $(DEFINE_FLAGS)
LD_FLAGS	= -fPIC -shared -lpthread

#--------- END OF ENVIRONMENT SETTING -------------
SRC = \
aec.c      buffer.c   filterbank.c  kiss_fft.c   mdf.c  powf_approach.c  smallft.c \
aud_mem.c  fftwrap.c  jitter.c      kiss_fftr.c  ns.c   preprocess.c aud_aec_api.c aud_ns_api.c

uclibc=$(shell echo $(CROSS_COMPILE)|grep uclib)
ifeq ($(uclibc),)
    ENV_LIBC=glibc
else
    ENV_LIBC=uclibc
endif

OBJ = $(SRC:.c=.o)

ifeq ("$(wildcard *.c */*.c)","")
all:
	@echo "nothing to be done for '$(OUTPUT_NAME)'"
clean:
	@echo "nothing to be done for '$(OUTPUT_NAME)'"
else
all: $(OBJ)
	@echo Creating library $(MODULE_NAME) ...
	@$(CC) -o $(MODULE_NAME).so $(OBJ) $(LD_FLAGS)
	@$(AR) rcs $(MODULE_NAME).a $(OBJ)

%.o:%.c
	@echo Compiling $<
	@$(CC) $(C_CFLAGS) $< -c -o $@

clean:
	@rm -f $(OBJ) *.o *.a *.so*
	@rm -rf $(OUTPUT_DIR)
endif

install:
	@echo Installing Library $(MODULE_NAME) ...
	@mkdir -p $(OUTPUT_DIR)
	@mkdir -p $(LIB_INC_PATH)
	@cp -af *.so* *.a $(OUTPUT_DIR)
	@cp -af ../include/* $(LIB_INC_PATH)
ifneq ("$(wildcard prebuild/$(ENV_LIBC)/*.a)","")
	@cp -avf prebuild/$(ENV_LIBC)/*.* $(OUTPUT_DIR)
endif

###############################################################################
# rtos Makefile                                                               #
###############################################################################
else ifeq ($(NVT_PRJCFG_CFG),rtos)
#--------- ENVIRONMENT SETTING --------------------
C_CFLAGS = $(PLATFORM_CFLAGS) $(EXTRA_INCLUDE)
#--------- END OF ENVIRONMENT SETTING -------------
LIB_NAME = $(MODULE_NAME).a
SRC = \


OBJ = $(SRC:.c=.o)

all: $(LIB_NAME)

$(LIB_NAME): $(OBJ)
	@echo Creating $@...
	@$(AR) rcsD $@ $(OBJ)
	@$(BUILD_DIR)/nvt-tools/nvt-ld-op --arc-sha1 $@

%.o: %.c
	@echo Compiling $<
	@$(CC) $(C_CFLAGS) -c $< -o $@

clean:
	@rm -f $(LIB_NAME) $(OBJ) $(LIB_NAME).sym *.o *.a *.so*
	@rm -rf $(OUTPUT_DIR)

install:
	@echo Installing Library $(MODULE_NAME) ...
	@mkdir -p $(OUTPUT_DIR)
	@mkdir -p $(LIB_INC_PATH)
	@cp -avf $(LIB_NAME) $(OUTPUT_DIR)
endif
