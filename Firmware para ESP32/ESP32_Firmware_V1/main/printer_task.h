// printer_task.h
#ifndef PRINTER_TASK_H
#define PRINTER_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "printer_config.h"

// CONFIGURACIÓN DE LA TAREA

#define TASK_STACK_SIZE     4096    // Reducido de 4096
#define TASK_PRIORITY       5
#define TASK_NAME           "printer_task"

// ESTRUCTURAS ESPECÍFICAS DE TAREA

typedef struct {
    printer_state_t state;
    uint32_t error_count;
    uint32_t success_count;
    uint32_t last_response_time;
    uint8_t reconnect_attempts;
    bool initialized;
} task_context_t;

// PROTOTIPOS PÚBLICOS

// Gestión de la tarea
bool printer_task_start(void);
void printer_task_stop(void);
void printer_task_suspend(void);
void printer_task_resume(void);

// Funciones de procesamiento
void interpretar_estado(uint8_t sts1, uint8_t sts2);
void procesar_status_s1(const uint8_t* data, size_t len);

// Sistema de reconexión
bool printer_reconnect(void);
void printer_reset_errors(void);
bool printer_needs_reconnection(void);
void printer_handle_error(printer_error_t error);

// Comunicación UART
bool send_enq_command(void);
bool send_status_s1_command(void);
int receive_response(uint8_t* buffer, size_t buffer_size, uint32_t timeout_ms);

// Monitoreo
void print_task_stats(void);
size_t get_task_stack_high_water_mark(void);

#endif // PRINTER_TASK_H