#include "stdio.h"
#include "tchar.h"
#include "Linux4Win.h"

/**
 * @brief Linux��shmget�V�X�e���R�[����Windows�p�Ƀ|�[�e�B���O
 * @return ���L�������A�N�Z�X�p�n���h��
 */
shm_key_t shmget_win(int key, size_t size)
{
	TCHAR	key_str[8];

	memset(key_str, 0, sizeof(key_str));
	_stprintf(key_str, _T("0x%04x"), key);
	return CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, key_str);
}
