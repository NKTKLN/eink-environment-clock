/* Captive-portal DNS: answers every A query with the access point address
 * so phones open the setup page on their own. Runs only while the AP is up. */
#pragma once

void dns_server_start(void);
void dns_server_stop(void);
