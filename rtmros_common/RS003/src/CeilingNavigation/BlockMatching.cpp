#include "BlockMatching.h"
#include "CeilingMap.h"
#ifdef _DEBUG
#ifdef WIN32
#include <crtdbg.h>
#endif
#endif

#ifdef _USE_SHOW_MAP
#include "Common/sharedmem.h"
extern robot_status *rstat;
#endif

#define SEARCH_RANGE	4
#define SIZE_HALF	(block_size / 2) //���W���[���ԋ��ʂ̒l�D���Ƃ����ׂ�
#define ANGLE_MAX	(m_Resolution * 2 + 1)

/**
 *@brief �R���X�g���N�^
 */
BlockMatching::BlockMatching(void) : m_BlockTable(0)
{
}

/**
 *@brief �f�X�g���N�^
 *@note �摜�̉�]�E�k���e�[�u���̈��j������
 */
BlockMatching::~BlockMatching(void)
{
	if(m_BlockTable != 0)
		DeleteBlockTable();
}

/**
 * @brief ��]�E�k���z����쐬����
 * @param center_x �摜��]���SX���W�B-1�̏ꍇ�͉摜�̒��S���g�p����B(�f�t�H���g-1)
 * @param center_y �摜��]���SY���W�B-1�̏ꍇ�͉摜�̒��S���g�p����B(�f�t�H���g-1)
 * @param image_width ���͉摜��
 * @param image_height ���͉摜����
 * @param block_size �ŏI�I�ȉ摜�T�C�Y(�c�E���T�C�Y)
 * @param Coefficient �摜�k���p�W��
 * @param Resolution ��]����\(���x���݂ŉ�]�摜�𐶐����邩)
 * @param BlackWhiteValue 2�l����臒l
 *
 */
void BlockMatching::CreateBlockMap(const long center_x,
						   const long center_y,
						   const unsigned long image_width,
						   const unsigned long image_height,
						   const unsigned short block_size,
						   const double Coefficient,
						   const unsigned short Resolution,
						   const unsigned short BlackWhiteValue)
{
	int		i;
	int		x,	y;
	int		x2,	y2;
	int		sx,	sy;
	int		degree_min, degree_max;
	double	cx,	cy, dx1, dy1;
	unsigned long width, height;

	/*
		���S�_����`����Ă��Ȃ��ꍇ�͉摜�f�[�^�̒��S�_���̗p����
		(�ŏI�I�Ȓ��S���W�Ƃ��邽��1/4�T�C�Y�̒��S�ɂ���)
	*/

	width = image_width / 2;
	if(center_x == -1)
		cx = width / 2;
	else
		cx = center_x;	/* �J�����Z���^�[�ݒ� */

	height = image_height / 2;
	if(center_y == -1)
		cy = height / 2;
	else
		cy = center_y;	/* �J�����Z���^�[�ݒ� */

	sx = cx - (int)((double)SIZE_HALF / Coefficient);
	sy = cy - (int)((double)SIZE_HALF / Coefficient);

	m_Resolution = 360 / 2 / Resolution;
	degree_max = m_Resolution; //��]�p�̃C���f�b�N�X�̐�(���̕���)
	degree_min = m_Resolution * -1;//��]�p�̃C���f�b�N�X�̐�(���̕���)

	m_LatticeSize	= block_size;
	m_BlackWhiteValue = BlackWhiteValue;

	CreateBlockTable();

	for (y = 0; y < block_size; y++){
		for (x = 0; x < block_size; x++){
			for (i = degree_min; i <= degree_max; i++){
				x2 = sx + ( (double)((double)( x - SIZE_HALF )) * cos((double)i * M_PI / (double)m_Resolution) - 
							(double)((double)( y - SIZE_HALF )) * sin((double)i * M_PI / (double)m_Resolution) + (double)SIZE_HALF ) / Coefficient;

				y2 = sy + ( (double)((double)( x - SIZE_HALF )) * sin((double)i * M_PI / (double)m_Resolution) +
							(double)((double)( y - SIZE_HALF )) * cos((double)i * M_PI / (double)m_Resolution) + (double)SIZE_HALF ) / Coefficient;

/*				/\* ���U���W�ɕϊ��ɂ��Ă���łȂ���... *\/ */

				dx1 = x2;
				dy1 = y2;

				if (dx1 <   0 )			  dx1 = 0;
				if (dx1 > (width - 1) )	  dx1 = (width - 1);
				if (dy1 <   0 )			{ dy1 = 0;					dx1=0; }
				if (dy1 > (height - 1) ){ dy1 = (height - 1); dx1=0; }

				m_BlockTable[ i + m_Resolution][x][y] = (int)((int)dx1 + (int)dy1 * (image_width / 2));
			}
		}
	}
}

/**
 *@brief CreateBlockMap��block_sizede�����Resolution�Ŏw�肳�ꂽ�傫���̔z��𐶐�����B
 */
void BlockMatching::CreateBlockTable()
{
	m_BlockTable = new long**[ANGLE_MAX];
	for(int cnt_z = 0; cnt_z < ANGLE_MAX ; cnt_z++){
		m_BlockTable[cnt_z] = new long*[m_LatticeSize];
		for(int cnt_x_y = 0; cnt_x_y < m_LatticeSize; cnt_x_y++){
			m_BlockTable[cnt_z][cnt_x_y] = new long[m_LatticeSize];
		}
	}
}

/**
 *@brief CreateBlockTable()�Ő��������̈��j������B
 */
void BlockMatching::DeleteBlockTable()
{
	for(int cnt_z = 0; cnt_z < ANGLE_MAX ; cnt_z++){
		for(int cnt_x_y = 0; cnt_x_y < m_LatticeSize; cnt_x_y++){
			delete[] m_BlockTable[cnt_z][cnt_x_y];
		}
		delete[] m_BlockTable[cnt_z];
	}
	delete[] m_BlockTable;
	m_BlockTable = 0;
}

/**
 * @brief �}�b�v�ǐՂ��s���B
 * @param CeilingMap �}�b�`���O���s���x�[�X�摜�ւ̎Q��
 * @param CurImage ���݂̃C���[�W�ւ̎Q��
 *
 */
void BlockMatching::MapTracking(CeilingMap &CeilingMap, ImageData &CurImage)
{
	ImageData		capImage;
	SignedOdometry_st	odometry;
#ifdef _USE_SHOW_MAP
	int				x,		y;
#endif
	static int reach = 0; //�}�b�`���O�̒T���͈͂𒲐�����p�����[�^�Ǝv���邪�Ƃ肠�����s�g�p��

	/* �}�b�v�}�b�`���O */
	/* �}�b�`���O����ɂ́A�L���v�`���摜��1/4�X�P�[���ɏk�߂�K�v������ */
	capImage = CurImage.PyrDown();

	odometry = Matching(CeilingMap, capImage, reach);
	CeilingMap.IncrementOdometry(odometry);

#ifdef _USE_SHOW_MAP
	ImageData		PackImage;
	PackImage.CreateImage(m_LatticeSize, m_LatticeSize);
	Packing(capImage, PackImage, 0/*CeilingMap.GetTheta()*/);//�����ŉ摜����]���Ă���

	for (y = 0; y < m_LatticeSize; y++) {
		for (x = 0; x < m_LatticeSize; x++) {
			rstat->buf[x+y*m_LatticeSize] = (PackImage[x+y*m_LatticeSize]);
		}
	}  
	rstat->x = CeilingMap.GetPosX();
	rstat->y = CeilingMap.GetPosY();
	rstat->r = CeilingMap.GetTheta();
#endif
}

/**
 *@brief �摜�}�b�`���O���s���B
 *@param CeilingMap �}�b�`���O���s���x�[�X�摜�ւ̎Q��
 *@param CurImage ���݉摜�ւ̎Q��
 *@param reach �}�b�`���O�͈͕␳�l
 */
SignedOdometry_st BlockMatching::Matching(CeilingMap &CeilingMap, ImageData &CurImage, int reach)
{
	SignedOdometry_st	result;
	ImageData		PackImage;
	int				min;
	int				i,		s;
	int				x,		y;

	result.m_x = 0;
	result.m_y = 0;
	result.m_theta = 0.0;

	PackImage.CreateImage(m_LatticeSize, m_LatticeSize);
	Packing(CurImage, PackImage, CeilingMap.GetTheta());

	// �]����l�����߂Ă����E�E�E���݂̈ʒu���ǂ��l��T������
	min = SumOfAbsoluteDifference(CeilingMap, PackImage, CeilingMap.GetPosX(), CeilingMap.GetPosY());

	//�p�x�����T�� �}m_Resolution * SEARCH_RANGE
	for ( i = -(SEARCH_RANGE + reach); i <= (SEARCH_RANGE + reach); i++) { 

		Packing(CurImage, PackImage, CeilingMap.GetTheta() + (double)( i * M_PI / m_Resolution ));
		//X�����T�� �}SEARCH_RANGE
		for ( x = -(SEARCH_RANGE + reach); x <= (SEARCH_RANGE + reach); x++) {

			//Y�����T�� �}SEARCH_RANGE
			for ( y = -(SEARCH_RANGE + reach); y <= (SEARCH_RANGE + reach); y++) {
				s = SumOfAbsoluteDifference(CeilingMap, PackImage,  CeilingMap.GetPosX() + x, CeilingMap.GetPosY() + y);

				if (s < min) {
					//���ݒn�̍X�V
					min = s;
					result.m_x = x;
					result.m_y = y;
					result.m_theta = i;
				}
			}
		}
	}
	result.m_theta = result.m_theta * M_PI / m_Resolution;

	return result;
}


/**
 *@brief		�P�x���̑��a(SAD; Sum of Absolute Difference)���v�Z����
 *@attention	��r��(�W���ł�)64�~64�ōs���̂ŁA�摜�f�[�^��64�~64�ɂ��Ă����K�v������
 *@note			2�̉摜�̓�����W�_�̍�����ώZ����B
 *@param		src �}�b�`���O���s���x�[�X�摜�̎Q��
 *@param		data 64�~64�k���ς݌��݉摜�̎Q��
 *@param		increment_x �x�[�X�摜���_����̃I�t�Z�b�g
 *@param		increment_y �x�[�X�摜���_����̃I�t�Z�b�g
 */
int BlockMatching::SumOfAbsoluteDifference(ImageData &src, ImageData &data, unsigned long offset_x, unsigned long offset_y)
{
	int cell_x, cell_y;
	int result, diff;

	result = 0;
	if ( src.GetWidth() == 0 || src.GetHeight() == 0 || data.GetWidth() == 0 || data.GetHeight() == 0 ) {
	  fprintf(stderr, "Warning : %ssrc or data image is 0 size\n", __PRETTY_FUNCTION__);
	  return result;
	}
	for( cell_y = 0; cell_y < m_LatticeSize; cell_y++) {
		for( cell_x = 0; cell_x < m_LatticeSize; cell_x++) {
			diff = src(cell_x + offset_x, cell_y + offset_y) - data(cell_x, cell_y);
			if ( diff < 0)
				result -= diff;
		}
	}
	return result;
}

/**
 *@brief �摜��64�~64�ɏk������
 *@param org �k�����摜�̎Q��



 *@param packData 64�~64�k���f�[�^�i�[��̎Q��
 *@param theta �摜�̉�]�p�x
 */
void BlockMatching::Packing(ImageData &org, ImageData &packData, double theta)
{
	int x, y;
	int rotation;

	if (theta > M_PI)		/* 180�x���z���Ă���ꍇ�A�}�C�i�X�ɕύX���� */
		theta = -M_PI * 2 + theta;
	else if (theta < -M_PI)	/* -180�������ꍇ�A�v���X�ɕύX���� */
		theta = M_PI * 2 + theta;

	rotation = (int)(theta / ( M_PI / m_Resolution ) ) + m_Resolution;	/* ���W�A��->�x �ϊ� */

	for (x=0; x < m_LatticeSize; x++) {
		for (y=0; y < m_LatticeSize; y++) {
			if (m_BlockTable[rotation][x][y] >= 0 && m_BlockTable[rotation][x][y] < org.GetWidth() * org.GetHeight())
				packData[x+y*m_LatticeSize] = org[m_BlockTable[rotation][x][y]];	/* ��]�����ꍇ�̍��W�ʒu�ɂ���f�[�^�����o�� */
			else
				packData[x+y*m_LatticeSize] = 0;

			if (packData[x+y*m_LatticeSize] > m_BlackWhiteValue )	/* 2�l����臒l������s���A��/���ɕύX���� */
				packData[x+y*m_LatticeSize] = MAX_BRIGHTNESS;
			else
				packData[x+y*m_LatticeSize] = 0;
		}
	}
}
