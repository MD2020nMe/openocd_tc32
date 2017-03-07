#ifndef TC32_H
#define TC32_H

#include "embeddedice.h"

int tc32_init_arch_info(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap);
int tc32_init_target(struct command_context *cmd_ctx,
		struct target *target);

#endif /* TC32_H */
