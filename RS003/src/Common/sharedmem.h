#ifndef _SHAREDMEM_H_
#define _SHAREDMEM_H_


#define SHM_KEY 16000


typedef struct robot_status
{
  int		x;
  int		y;
  double	r;
  int		s;
  unsigned char buf[64*64];
  unsigned char buf2[64*64];
} robot_status;

#ifdef __cplusplus
	extern "C" {	/* C++�Ŏg�p����ꍇ�֐��C���������Ȃ��悤�� */
#endif
unsigned char *InitSharedMem(int key, int size);

#ifdef __cplusplus
}
#endif


#endif

