CC = gcc
CFLAGS = -Wall -Wextra -Iinclude -Itest
OBJDIR = build
BINDIR = bin

# Lista de fontes
SRCS = \
  test/test_main.c \
  test/unity.c \
  src/app_logic.c

# Gerar caminho de objetos dentro de build/
OBJS = $(SRCS:.c=.o)
OBJS := $(patsubst %, $(OBJDIR)/%, $(OBJS))

# Executável final
TARGET = $(BINDIR)/test_runner

# Regra padrão
all: $(TARGET)

# Regra para compilar .o
$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Link final
$(TARGET): $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $^ -o $@ -lm

# Testes
run: $(TARGET)
	./$(TARGET)

# Limpar
clean:
	rm -rf $(OBJDIR) $(BINDIR)

