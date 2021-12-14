MODULE_NAME = faac
# DIRs
OUTPUT_DIR = $(LIBRARY_DIR)/output
# INCs
LIB_INC_PATH = $(LIBRARY_DIR)/include/faac
# INC FLAGs
EXTRA_INCLUDE += -I$(LIB_INC_PATH)

INSTALL_DIR := $(shell pwd)/output

.PHONY: all clean

FAAC := faac-1_30

check_exist = if [ ! -e ./.nvt_finish ]; \
              then rm -rf $(1); \
              curl -s -L -o faac-1_30.tar.gz https://github.com/knik0/faac/archive/refs/tags/1_30.tar.gz; \
              tar -zxf $(1).tar.gz; \
              fi

all:
	@echo Creating library $(MODULE_NAME) ...
	@if [ ! -f ./.nvt_finish ]; then \
	$(call check_exist, ${FAAC}); \
	cd ${FAAC}; ./bootstrap; ./configure --prefix=${INSTALL_DIR} CC=${CC} CXX=${CXX} AR=${AR} --host=${NVT_HOST} --target=${NVT_HOST}; make -j$(MULTI_CORES); \
	cd -; \
	fi
	@cd ${FAAC}; make install; cd -;
	@touch ./.nvt_finish

clean:
	@make -C ${FAAC} clean
	@rm -rf $(OUTPUT_DIR) $(INSTALL_DIR)

install:
	@mkdir -p $(OUTPUT_DIR)
	@mkdir -p $(LIB_INC_PATH)
	@cp -avf $(INSTALL_DIR)/lib/*.so* $(INSTALL_DIR)/lib/*.a $(OUTPUT_DIR)
	@cp -avf $(INSTALL_DIR)/include/* $(LIB_INC_PATH)
