/*
 * Version comparison and GitHub release parsing (pure, host-tested).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Semantic versions, an optional leading "v" is ignored:
 *   1.2.0 < 1.10.0,   1.2.0-rc1 < 1.2.0,   build metadata (+...) ignored.
 * Returns <0, 0, >0 like strcmp. Unparsable versions sort lowest. */
int version_compare(const char *a, const char *b);

typedef struct {
    char version[32];   /* tag without a leading "v" */
    char url[256];      /* firmware.bin download URL */
    char page[160];     /* release page on GitHub */
    char notes[512];    /* release notes, cut to fit */
    int size;           /* asset size in bytes */
    bool prerelease;
} release_t;

/* Parses the JSON of GET /repos/{owner}/{repo}/releases/latest. Returns
 * false if there is no tag or no asset called asset_name. */
bool release_parse(const char *json, size_t len, const char *asset_name,
                   release_t *out);
