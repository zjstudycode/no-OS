EXEC = $(shell basename $(CURDIR))
SYMBOLS = 
INC = -I./include/ -I./projects/drivers/util/ \
-I./drivers/adc/ad9208/ad9208_api/ \
-I./drivers/dac/ad917x/ad917x_api/ \
-I./common_drivers/adc_core/ 

CFLAGS = -Werror -Wextra -O0 $(SYMBOLS) $(INC) -lm

all: $(EXEC)

$(EXEC): ./drivers/make/main.c \
	./drivers/platform/generic/i2c.c \
	./drivers/platform/generic/spi.c \
	./drivers/platform/generic/delay.c \
	./drivers/platform/generic/gpio.c \
	./projects/drivers/util/util.c \
	$(ARG1) 
	$(CC) $+ $(CFLAGS) -o $(EXEC) 

clean:
	rm -rf $(EXEC)
