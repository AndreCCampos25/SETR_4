/* auto-generated by gen_syscalls.py, don't edit */
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#endif
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
#include <syscalls/socket.h>

extern ssize_t z_vrfy_zsock_sendto(int sock, const void * buf, size_t len, int flags, const struct sockaddr * dest_addr, socklen_t addrlen);
uintptr_t z_mrsh_zsock_sendto(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	ssize_t ret = z_vrfy_zsock_sendto(*(int*)&arg0, *(const void **)&arg1, *(size_t*)&arg2, *(int*)&arg3, *(const struct sockaddr **)&arg4, *(socklen_t*)&arg5)
;
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic pop
#endif
