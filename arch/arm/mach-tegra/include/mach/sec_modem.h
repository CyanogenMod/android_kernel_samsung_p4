#ifndef _SEC_MODEM_H_
#define _SEC_MODEM_H_

#if defined(CONFIG_LINK_DEVICE_HSIC) || defined(CONFIG_LINK_DEVICE_USB)
void set_host_states(struct platform_device *pdev, int type);
void set_slave_wake(void);
int get_cp_active_state(void);
#else
#define set_host_states(pdev, type) do { } while (0);
#define set_slave_wake() do { } while (0);
#define get_cp_active_state(void) do {} while (0);
#endif

#ifdef CONFIG_SAMSUNG_LPM_MODE
extern int charging_mode_from_boot;
#endif

#endif /*_SEC_MODEM_H_*/
