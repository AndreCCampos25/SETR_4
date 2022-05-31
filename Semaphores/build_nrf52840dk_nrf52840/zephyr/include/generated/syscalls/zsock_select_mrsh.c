/* auto-generated by gen_syscalls.py, don't edit */
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#endif
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
#include <syscalls/socket_select.h>

extern int z_vrfy_zsock_select(int nfds, zsock_fd_set * readfds, zsock_fd_set * writefds, zsock_fd_set * exceptfds, struct zsock_timeval * timeout);
uintptr_t z_mrsh_zsock_select(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg5;	/* unused */
	int ret = z_vrfy_zsock_select(*(int*)&arg0, *(zsock_fd_set **)&arg1, *(zsock_fd_set **)&arg2, *(zsock_fd_set **)&arg3, *(struct zsock_timeval **)&arg4)
;
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic pop
#endif
