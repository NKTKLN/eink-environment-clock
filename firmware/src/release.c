#include "release.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

typedef struct {
    long num[3];
    const char *pre; /* points into the string, NULL for a release */
    size_t pre_len;
    bool ok;
} semver_t;

static semver_t parse(const char *s)
{
    semver_t v = { { 0, 0, 0 }, NULL, 0, false };
    if (s == NULL) {
        return v;
    }
    if (*s == 'v' || *s == 'V') {
        s++;
    }
    for (int i = 0; i < 3; i++) {
        if (!isdigit((unsigned char)*s)) {
            return v;
        }
        char *end;
        v.num[i] = strtol(s, &end, 10);
        s = end;
        if (i < 2) {
            if (*s != '.') {
                /* "1" or "1.2" are fine: missing parts are 0 */
                if (*s == '\0' || *s == '-' || *s == '+') {
                    break;
                }
                return v;
            }
            s++;
        }
    }
    if (*s == '-') {
        v.pre = s + 1;
        v.pre_len = strcspn(v.pre, "+");
    } else if (*s != '\0' && *s != '+') {
        return v;
    }
    v.ok = true;
    return v;
}

/* Pre-release identifiers compared dot by dot, numbers numerically. */
static int compare_pre(const char *a, size_t al, const char *b, size_t bl)
{
    while (al > 0 && bl > 0) {
        size_t an = 0, bn = 0;
        while (an < al && a[an] != '.') an++;
        while (bn < bl && b[bn] != '.') bn++;

        bool a_num = an > 0, b_num = bn > 0;
        for (size_t i = 0; i < an; i++) a_num &= isdigit((unsigned char)a[i]) != 0;
        for (size_t i = 0; i < bn; i++) b_num &= isdigit((unsigned char)b[i]) != 0;

        int c;
        if (a_num && b_num) {
            long x = strtol(a, NULL, 10), y = strtol(b, NULL, 10);
            c = (x > y) - (x < y);
        } else if (a_num != b_num) {
            c = a_num ? -1 : 1; /* numeric identifiers sort first */
        } else {
            size_t n = an < bn ? an : bn;
            c = strncmp(a, b, n);
            if (c == 0) {
                c = (an > bn) - (an < bn);
            }
        }
        if (c != 0) {
            return c;
        }
        a += an; al -= an;
        b += bn; bl -= bn;
        if (al > 0) { a++; al--; }
        if (bl > 0) { b++; bl--; }
    }
    return (al > 0) - (bl > 0);
}

int version_compare(const char *a, const char *b)
{
    semver_t x = parse(a), y = parse(b);
    if (!x.ok || !y.ok) {
        return (x.ok > y.ok) - (x.ok < y.ok);
    }
    for (int i = 0; i < 3; i++) {
        if (x.num[i] != y.num[i]) {
            return x.num[i] > y.num[i] ? 1 : -1;
        }
    }
    if (x.pre == NULL || y.pre == NULL) {
        /* a release is newer than any of its pre-releases */
        return (x.pre == NULL) - (y.pre == NULL);
    }
    int c = compare_pre(x.pre, x.pre_len, y.pre, y.pre_len);
    return (c > 0) - (c < 0);
}

static void copy(char *dst, size_t size, const char *src)
{
    snprintf(dst, size, "%s", src != NULL ? src : "");
    /* do not leave half a UTF-8 character at the end */
    size_t len = strlen(dst);
    if (len == size - 1) {
        while (len > 0 && ((unsigned char)dst[len - 1] & 0xC0) == 0x80) {
            len--;
        }
        if (len > 0 && ((unsigned char)dst[len - 1] & 0xC0) == 0xC0) {
            len--;
        }
        dst[len] = '\0';
    }
}

bool release_parse(const char *json, size_t len, const char *asset_name,
                   release_t *out)
{
    memset(out, 0, sizeof(*out));
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        return false;
    }

    bool ok = false;
    const cJSON *tag = cJSON_GetObjectItemCaseSensitive(root, "tag_name");
    const cJSON *assets = cJSON_GetObjectItemCaseSensitive(root, "assets");
    if (!cJSON_IsString(tag) || !cJSON_IsArray(assets)) {
        goto done;
    }

    const char *t = tag->valuestring;
    if (*t == 'v' || *t == 'V') {
        t++;
    }
    copy(out->version, sizeof(out->version), t);
    copy(out->page, sizeof(out->page),
         cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(root, "html_url")));
    copy(out->notes, sizeof(out->notes),
         cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(root, "body")));
    out->prerelease = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "prerelease"));

    const cJSON *a;
    cJSON_ArrayForEach(a, assets) {
        const char *name =
            cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(a, "name"));
        const char *url = cJSON_GetStringValue(
            cJSON_GetObjectItemCaseSensitive(a, "browser_download_url"));
        if (name != NULL && url != NULL && strcmp(name, asset_name) == 0 &&
            strlen(url) < sizeof(out->url)) {
            copy(out->url, sizeof(out->url), url);
            const cJSON *size = cJSON_GetObjectItemCaseSensitive(a, "size");
            out->size = cJSON_IsNumber(size) ? (int)size->valuedouble : 0;
            ok = true;
            break;
        }
    }

done:
    cJSON_Delete(root);
    return ok;
}
