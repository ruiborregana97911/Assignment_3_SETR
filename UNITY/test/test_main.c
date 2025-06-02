#include "unity.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>  
#include "unity.h"
#include "app_logic.h"  // Inclui os protótipos e a variável RTDB


// Mock UART output buffer
char uart_output_log[256] = {0};

// Sobrescreve a função fraca definida em app_logic.c
void send_uart_msg(const char *msg) {
    strncat(uart_output_log, msg, sizeof(uart_output_log) - strlen(uart_output_log) - 1);
}


void setUp(void) {
    memset(&RTDB, 0, sizeof(RTDB));        // limpa base de dados
    uart_output_log[0] = '\0';            // limpa UART log
}


void tearDown(void) {}






// ---------------------------
// Testes para calc_checksum
// ---------------------------



void test_calc_checksum_basico(void) {
    const char *cmd = "A";
    const char *data = "BC";
    uint8_t expected = 198 % 256;
    TEST_ASSERT_EQUAL_UINT8(expected, calc_checksum(cmd, data));
}

void test_calc_checksum_com_overflow(void) {
    const char *cmd = "~";
    const char *data = "~D";
    uint8_t expected = (126 + 126 + 68) % 256; // 320 % 256 = 64
    TEST_ASSERT_EQUAL_UINT8(expected, calc_checksum(cmd, data));
}

void test_calc_checksum_vazio(void) {
    const char *cmd = "X";
    const char *data = "";
    uint8_t expected = 88;
    TEST_ASSERT_EQUAL_UINT8(expected, calc_checksum(cmd, data));
}

void test_calc_checksum_multicaracter_cmd(void) {
    const char *cmd = "AB";
    const char *data = "C";
    uint8_t expected = (65 + 67) % 256;
    TEST_ASSERT_EQUAL_UINT8(expected, calc_checksum(cmd, data));
}

// ---------------------------
// Testes para print_float_as_string
// ---------------------------


void test_print_float_positive(void) {
    char buf[16];
    print_float_as_string(buf, sizeof(buf), 12.3f);
    TEST_ASSERT_EQUAL_STRING("12.3", buf);
}

void test_print_float_negative(void) {
    char buf[16];
    print_float_as_string(buf, sizeof(buf), -4.7f);
    TEST_ASSERT_EQUAL_STRING("-4.7", buf);
}

void test_print_float_zero(void) {
    char buf[16];
    print_float_as_string(buf, sizeof(buf), 0.0f);
    TEST_ASSERT_EQUAL_STRING("0.0", buf);
}

void test_print_float_rounding_behavior(void) {
    char buf[16];
    print_float_as_string(buf, sizeof(buf), 2.96f);
    TEST_ASSERT_EQUAL_STRING("3.0", buf);
}

void test_print_float_buffer_small(void) {
    char buf[4];
    print_float_as_string(buf, sizeof(buf), 1.2f);
    TEST_ASSERT_TRUE(strlen(buf) < sizeof(buf));
}

void test_print_float_negative_small_fraction(void) {
    char buf[16];
    print_float_as_string(buf, sizeof(buf), -1.05f);
    TEST_ASSERT_EQUAL_STRING("-1.0", buf);
}

// ---------------------------
// Testes para cmd
// ---------------------------


void test_frame_invalido_formato(void) {
    process_frame("BLAH!");

    char expected[32];
    snprintf(expected, sizeof(expected), "\r\n#Ef%03d!\r\n", calc_checksum("E", "f"));
    TEST_ASSERT_EQUAL_STRING(expected, uart_output_log);
}

void test_checksum_invalido(void) {
    process_frame("#M075999!");  // checksum propositadamente errado

    char expected[32];
    snprintf(expected, sizeof(expected), "\r\n#Es%03d!\r\n", calc_checksum("E", "s"));
    TEST_ASSERT_EQUAL_STRING(expected, uart_output_log);
}

void test_cmd_M_max_temp_valido(void) {
    const char *temp_str = "075";
    char frame[32];
    uint8_t cs = calc_checksum("M", temp_str);
    snprintf(frame, sizeof(frame), "#M%s%03d!", temp_str, cs);

    process_frame(frame);

    char expected_ack[32];
    snprintf(expected_ack, sizeof(expected_ack), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
    TEST_ASSERT_TRUE(strstr(uart_output_log, expected_ack) != NULL);
    TEST_ASSERT_TRUE(strstr(uart_output_log, "Max temp = 75") != NULL);
}

void test_cmd_S_pid_valido(void) {
    const char *pid_data = "p123i045d078";
    uint8_t cs = calc_checksum("S", pid_data);
    char frame[64];
    snprintf(frame, sizeof(frame), "#S%s%03d!", pid_data, cs);

    process_frame(frame);

    TEST_ASSERT_FLOAT_WITHIN(0.1, 12.3f, RTDB.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 4.5f, RTDB.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 7.8f, RTDB.kd);

    char expected[32];
    snprintf(expected, sizeof(expected), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
    TEST_ASSERT_TRUE(strstr(uart_output_log, expected) != NULL);
}

void test_cmd_S_pid_invalido_mal_formado(void) {
    const char *bad_data = "x123i045d078";
    uint8_t cs = calc_checksum("S", bad_data);
    char frame[64];
    snprintf(frame, sizeof(frame), "#S%s%03d!", bad_data, cs);

    process_frame(frame);

    char expected[32];
    snprintf(expected, sizeof(expected), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
    TEST_ASSERT_TRUE(strstr(uart_output_log, expected) != NULL);
}

void test_cmd_G_leitura_pid(void) {
    RTDB.kp = 1.0f;
    RTDB.ki = 2.0f;
    RTDB.kd = 3.0f;
    char frame[16];
    uint8_t cs = calc_checksum("G", "");
    snprintf(frame, sizeof(frame), "#G%03d!", cs);

    process_frame(frame);

    char pid_str[32];
    snprintf(pid_str, sizeof(pid_str), "p010i020d030");

    char expected[64];
    snprintf(expected, sizeof(expected), "\r\n#g%s%03d!\r\n", pid_str, calc_checksum("g", pid_str));
    TEST_ASSERT_TRUE(strstr(uart_output_log, expected) != NULL);
}


void test_cmd_C_leitura_temperatura(void) {
    RTDB.cur_temp = 42;
    char frame[16];
    uint8_t cs = calc_checksum("C", "");
    snprintf(frame, sizeof(frame), "#C%03d!", cs);

    process_frame(frame);

    char temp_str[8];
    snprintf(temp_str, sizeof(temp_str), "%03d", RTDB.cur_temp);

    char expected[64];
    snprintf(expected, sizeof(expected), "\r\n#c%s%03d!\r\n", temp_str, calc_checksum("c", temp_str));

    TEST_ASSERT_TRUE(strstr(uart_output_log, expected) != NULL);
}


int main(void) {
    UNITY_BEGIN();

    // Testes de checksum
    RUN_TEST(test_calc_checksum_basico);
    RUN_TEST(test_calc_checksum_com_overflow);
    RUN_TEST(test_calc_checksum_vazio);
    RUN_TEST(test_calc_checksum_multicaracter_cmd);

    // Testes de print_float_as_string
    RUN_TEST(test_print_float_positive);
    RUN_TEST(test_print_float_negative);
    RUN_TEST(test_print_float_zero);
    RUN_TEST(test_print_float_rounding_behavior);
    RUN_TEST(test_print_float_buffer_small);
    RUN_TEST(test_print_float_negative_small_fraction);

    // Testes de process_frame
    RUN_TEST(test_frame_invalido_formato);
    RUN_TEST(test_checksum_invalido);
    RUN_TEST(test_cmd_M_max_temp_valido);
    RUN_TEST(test_cmd_C_leitura_temperatura);
    RUN_TEST(test_cmd_S_pid_valido);
    RUN_TEST(test_cmd_S_pid_invalido_mal_formado);
    RUN_TEST(test_cmd_G_leitura_pid);

    return UNITY_END();
}
