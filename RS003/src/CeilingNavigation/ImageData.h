#pragma once

#include <rtm/idl/BasicDataTypeSkel.h>

#include "std_hdr.h"
#include "cv.h"
#include "highgui.h"

#define MAX_BRIGHTNESS	255		/* �C���[�W�f�[�^�������l	*/

using namespace RTC;

/**
 *@brief 2�l�C���[�W�f�[�^�Ǘ��N���X
 *@note �����ɂ́AOpenCV�̃f�v�X�r�b�g��IPL_DEPTH_8U�������̂ŃO���[�X�P�[���������Ă���B
 *@attention �J���[�C���[�W����͂�����ۂɂ́A���O�ɃO���[�X�P�[���ɕϊ����Ă������Ƃ𐄏�����B
 */
class ImageData
{
public:
	ImageData(void);					/* �R���X�g���N�^		*/
	~ImageData(void);					/* �f�X�g���N�^			*/
	ImageData(const ImageData& src);	/* �R�s�[�R���X�g���N�^ */

/* ���J�֐� */
public:
	void CreateImage(const unsigned long width, const unsigned long height);	/* 2�l�C���[�W�f�[�^�̈�쐬 				*/
	void Clear(unsigned char value = MAX_BRIGHTNESS);							/* �C���[�W�f�[�^�̈揉���� 				*/
	void DeleteImage();															/* �C���[�W�f�[�^�̈�j��					*/
	ImageData PyrDown();														/* �摜�̏�ݍ���							*/
	void SetData(TimedOctetSeq orgData);										/* �C���[�W�f�[�^�ϊ�						*/
	void SetData(IplImage *cvImage);											/* OpenCV�`���C���[�W��Ǝ��`���ɕϊ����� 	*/
	int	LoadImage(std::string fileName, int flags = CV_LOAD_IMAGE_ANYCOLOR);	/* �C���[�W�f�[�^�ǂݍ���					*/

	/**
	 *@brief �C���[�W�f�[�^�̕����擾���܂�
	 *@return �C���[�W�f�[�^�̕�
	 */
	unsigned long GetWidth()	{ return m_width; }								/* �C���[�W���̎擾							*/

	/**
	 *@brief �C���[�W�f�[�^�̍������擾���܂�
	 *@return �C���[�W�f�[�^�̍���
	 */
	unsigned long GetHeight()	{ return m_height; }							/* �C���[�W�����̎擾						*/

/* operator���� */
public:
#ifdef WIN32
	/**
	 *@brief �C���[�W�f�[�^�ւ̔z��`���A�N�Z�X
	 */
	unsigned char &operator[](size_t index)	{ return m_image[index];	}
#endif

	/**
	 *@brief �C���[�W�f�[�^�ւ̔z��`���A�N�Z�X
	 */
	unsigned char &operator[](int index)	{ return m_image[index];	}

	/**
	 *@brief �C���[�W�f�[�^�ւ�2�����z��`���A�N�Z�X
	 *@param x : X���W�ʒu
	 *@param y : Y���W�ʒu
	 */
	unsigned char &operator()(size_t x, size_t y)	{ return m_image[x + y * m_width];	}

	/**
	 *@brief �C���[�W�f�[�^�ւ̃|�C���^�擾
	 */
	operator unsigned char*()				{ return m_image;			}

	/**
	 *@brief �{�N���X�̑������
	 */
	ImageData& operator=(const ImageData& org);

private:
	unsigned long	m_width;		/* �C���[�W�f�[�^��						*/
	unsigned long	m_height;		/* �C���[�W�f�[�^����					*/
	unsigned char	*m_image;		/* �C���[�W�f�[�^�̈�					*/
	IplImage		*m_cvImage;		/* OpenCV�`���C���[�W�f�[�^�̈�			*/
	IplImage		*m_cvSimImage;	/* �[��OpenCV�`���C���[�W�f�[�^�̈�		*/
};
