/* Captive-portal DNS answer builder (pure, host-tested). */
#pragma once

#include <stdint.h>

/* Turns the query in buf[0..len) into a reply in place: A queries get one
 * answer with ip (network byte order), other types get an empty answer.
 * Returns the reply length, or -1 if the packet should be ignored. */
int dns_build_reply(uint8_t *buf, int len, int cap, uint32_t ip);
