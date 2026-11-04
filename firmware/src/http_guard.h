/*
 * Request checks against cross-site requests and DNS rebinding (pure,
 * host-tested). The API has no password, so a web page from another site
 * must not be able to change settings or upload firmware.
 */
#pragma once

#include <stdbool.h>

/* Host header the clock answers to: an IP literal, a single-label name or
 * a name in a local zone (.local, .lan, .home, .internal, .localdomain,
 * .home.arpa), with an optional port. Anything else means the request
 * came through a public DNS name pointing at the clock (DNS rebinding). */
bool http_host_allowed(const char *host);

/* A state-changing request is fine without an Origin header (curl, Home
 * Assistant, scripts) or when the Origin is the clock itself. */
bool http_origin_allowed(const char *origin, const char *host);
