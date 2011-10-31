#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ImageData.h"
#ifdef _DEBUG
#ifdef WIN32
#include <crtdbg.h>
#endif
#endif

/**
 *@brief �R���X�g���N�^
 *@note �C���[�W�f�[�^�p�|�C���^�A����щ摜�̕��E�������N���A����
 */
ImageData::ImageData(void) : 
	m_width(0), 
	m_height(0),
	m_image(0),
	m_cvImage(0),
	m_cvSimImage(0)

{
}

/**
 *@brief �f�X�g���N�^
 *@note �m�ۂ���Ă���C���[�W�f�[�^�̈��j������
 */
ImageData::~ImageData(void)
{
	DeleteImage();
}

/**
 *@brief �R�s�[�R���X�g���N�^
 *@param src �R�s�[��ImageData�C���X�^���X
 */
ImageData::ImageData(const ImageData& src)
{
	m_image			= 0;
	m_cvImage		= 0;
	m_cvSimImage	= 0;

	CreateImage(src.m_width ,src.m_height);
	memcpy(m_image, src.m_image, src.m_width * src.m_height);
}

/**
 *@brief �C���[�W�̏�ݍ��݂��s���B
 *@note ���摜��1/4�T�C�Y�ɏ�ݍ���ŐV����ImageData�����B
 *@return 1/4�T�C�Y�ɏ�ݍ��񂾉摜�f�[�^�B
 */
ImageData ImageData::PyrDown()
{
	ImageData		result;
	unsigned long	width, height;

	unsigned long h_cnt, w_cnt;
	unsigned long x_pos, y_pos;

	width	= m_width / 2;
	height	= m_height / 2;
	result.CreateImage(width, height);

	for ( h_cnt = 0; h_cnt < height; h_cnt++ ) {	/* dst�̍������[�v */
		for( w_cnt = 0; w_cnt < width; w_cnt++ ) {	/* dst�̕����[�v */
			x_pos = (w_cnt * 2);			/* 0,2,4,6,�E�E�Esrc�̊��s�N�Z�����Ԉ��� */
			y_pos = (h_cnt * 2) * m_width;	/* 0,2,4,6,�E�E�Esrc�̊�s�s�N�Z�����Ԉ��� */
			/*
				�摜�̉E�[����f�[�^�i�[�E�E�E
				�\�[�X�̃R�s�[�Ώۃs�N�Z���ƉE�E���E�E��(�Ԉ������s�N�Z��)�̃s�N�Z���l�̕��ς�����
				�E�E�E���[����E�[�Ɋi�[����̂ŋ����ɂȂ�
			*/
			/*
				�^�[�Q�b�g���W	= X
				�������ݑΏ�	= O

				+ + + + + + +
				+ + + + + + +
				+ + + +X+O+ +
				+ + + +O+O+ +
				+ + + + + + +
				+ + + + + + +
			*/
			result[ (int)(h_cnt * width + (width - w_cnt - 1)) ]
				= (unsigned int)( m_image[ y_pos + x_pos ] +	
								  m_image[ y_pos + x_pos + m_width] +
								  m_image[ y_pos + x_pos + 1 ] +
								  m_image[ y_pos + x_pos + 1 + m_width ] ) / 4;
		}
	}

	return result;
}

/**
 *@brief �C���[�W�f�[�^�`���ϊ�
 *@param orgData CameraEyeComp���瑗���Ă����摜�f�[�^��
 *@attention CameraEyeComp���瑗���Ă����f�[�^�����ϊ��ł��Ȃ��B
 */
void ImageData::SetData(TimedOctetSeq orgData)
{
	unsigned char	*imgData;

	DeleteImage();

	/* �o�C�i���z���K�؂ȏ�ԂɕύX���� */
	if(orgData.data.length() > 0){
		imgData = (unsigned char*)malloc(orgData.data.length());
		memcpy(imgData, (unsigned char*)(&(orgData.data[0])), orgData.data.length());
		m_cvSimImage = (IplImage *)imgData;
		m_cvSimImage->roi = NULL;
		m_cvSimImage->maskROI = NULL;
		m_cvSimImage->imageId = NULL;
		m_cvSimImage->tileInfo = NULL;
		m_cvSimImage->imageDataOrigin = NULL;
		m_cvSimImage->imageData = (char*)(&(imgData[sizeof(IplImage)]));
		SetData(m_cvSimImage);
	}
}

/**
 *@brief �C���[�W�f�[�^�̈�쐬
 *@param width ��������摜�̕�
 *@param height ��������摜�̍���
 */
void ImageData::CreateImage(const unsigned long width, const unsigned long height)
{
	m_width		= width;
	m_height	= height;

	DeleteImage();
	m_image = (unsigned char *)malloc( width * height );
	if(m_image == 0) {
		fprintf(stderr, "Can't malloc");
	}
	Clear(MAX_BRIGHTNESS);
}

/**
 *@brief �C���[�W�f�[�^�̈�����̒l�Ŗ��߂�
 *@param value ���ߍ��݂����f�[�^(�f�t�H���g�ł�0xFF)
 */
void ImageData::Clear(unsigned char value)
{
	if(m_image != 0)
		memset(m_image, value, m_width * m_height);
}

/**
 *@brief �C���[�W�f�[�^�̈���J������
 *
 */
void ImageData::DeleteImage()
{
	if(m_image != 0)
		free(m_image);
	m_image = 0;

	if(m_cvImage != 0)
		cvReleaseImage(&m_cvImage);
	m_cvImage = 0;

	if(m_cvSimImage)
		free(m_cvSimImage);
	m_cvSimImage = 0;
}

/**
 *@brief �C���[�W�f�[�^��ǂݍ��݁A2�l�摜�ɕϊ����܂��B
 *@note OpenCV��cvLoadImage�œǂݍ��݉\�Ȍ`���ł���Γǂݍ��݂܂��B
 *@param fileName �ǂݍ��ރC���[�W�t�@�C����
 *@param flags cvLoadImage�ɓn��flags�l
 *@attention flags�́A�{�֐�����CV_LOAD_IMAGE_ANYDEPTH���ݒ肳��܂��B
 *@return 0:����A-1:�ǂݍ��݃G���[
 */
int	ImageData::LoadImage(std::string fileName, int flags)
{
	/* �C���[�W�̈�̔j�����s���Ă��� */
	DeleteImage();

	m_cvImage = cvLoadImage(fileName.c_str(), flags | CV_LOAD_IMAGE_ANYDEPTH);
	if(m_cvImage == 0)	return -1;
	SetData(m_cvImage);

	return 0;
}

/**
 *@brief �C���[�W�f�[�^�`���ϊ�
 *@param cvImage OpenCV�`���f�[�^
 *@attention �S�p�^�[�����������킯�ł͂Ȃ��̂ŁA�ϊ��o�����ɗ����鎖�����邩���E�E�E�B
 */
void ImageData::SetData(IplImage *cvImage)
{
	IplImage	*grayImage = 0;
	char		*imageData;

	/* �摜�̕��E�������Ƃ肠�����i�[���Ă��� */
	m_width		= cvImage->width;
	m_height	= cvImage->height;
	imageData	= cvImage->imageData;

	if((cvImage->width * cvImage->height) != cvImage->imageSize){
		/* �C���[�W�f�[�^���J���[�摜 or �A���C�����g��������Ă��� */
		/* �ŏI�I��nChannels=1,depth=IPL_DEPTH_8U or 8S �ɂȂ�Ηǂ� */
		if(cvImage->nChannels != 1){
			/* �J���[�摜�炵�� */
			/* �O���[�X�P�[���ɕϊ����� */
			grayImage = cvCreateImage( cvGetSize(cvImage)  , IPL_DEPTH_8U, 1 );
			cvCvtColor(cvImage, grayImage, CV_BGR2GRAY);
			imageData = grayImage->imageData;
		}
		else{
			/* �O���[�X�P�[�� or ��l�摜�Ȃ̂ŁA�A���C�����g��������Ă��邾�� */
			m_width		= cvImage->widthStep;
			m_height	= cvImage->height;
		}

	}

	m_image		= (unsigned char *)malloc(m_width * m_height);
	memcpy(m_image, imageData, m_width * m_height);
	if(grayImage != 0)	cvReleaseImage(&grayImage);
}

/**
 *@brief �C���[�W�f�[�^�̑���I�y���[�^
 *@attention �{�I�y���[�^�ł́A�{�N���X�œƎ���`���Ă���C���[�W�f�[�^�̂ݑ������A���̑�2�`���̃C���[�W�f�[�^�͑������Ȃ��B
 */
ImageData& ImageData::operator=(const ImageData& org)
{
	CreateImage(org.m_width, org.m_height);
	memcpy(m_image, org.m_image, org.m_width * org.m_height);
	return *this;
}
