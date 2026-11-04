/* Health numbers for the log and /api/state: heap and task stack usage. */
#pragma once

#include "cJSON.h"

/* Writes one "health:" line to the log. */
void diag_log(void);

/* Adds "heap" and "tasks" objects to the given JSON object. */
void diag_to_json(cJSON *obj);
