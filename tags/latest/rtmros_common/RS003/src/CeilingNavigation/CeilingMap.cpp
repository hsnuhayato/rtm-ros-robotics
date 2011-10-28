#include "CeilingMap.h"

/**
 * @brief �R���X�g���N�^
 * @note �����͍s���Ă��Ȃ�
 */
CeilingMap::CeilingMap(void)
{
}

/**
 * @brief �f�X�g���N�^
 * @note �����͍s���Ă��Ȃ�
 */
CeilingMap::~CeilingMap(void)
{
}

/**
 * @brief �i�r�Q�[�V�����}�b�v�̓ǂݍ���
 * @param map_name �V��摜�t�@�C����
 */
void CeilingMap::ReadNavigationMap(std::string map_name)
{
	LoadImage(map_name);
}

/**
 * @brief ���W�_�̐ݒ���s��
 * @param x X���W
 * @param y Y���W
 * @param theta ����(���W�A��)
 */
void CeilingMap::SetLocation(unsigned long x, unsigned long y, double theta)
{
	m_CurOdometry.m_x		= x;
	m_CurOdometry.m_y		= y;
	m_CurOdometry.m_theta	= theta;
}

/**
 * @brief ���W�_�̐ݒ���s��
 * @param value ���W���
 * @note SetLocation(unsigned long x, unsigned long y, double theta)�̌Ăяo�����s��
 */
void CeilingMap::SetLocation(CeilingOdometry_st value)
{
	SetLocation(value.m_x, value.m_y, value.m_theta);
}

/**
 * @brief ���݂̍��W�����擾����B
 * @param ���W���
 */
CeilingOdometry_st	CeilingMap::GetLocation()
{
	return m_CurOdometry;
}

/**
 * @brief ���W�����X�V����
 * @note �{�֐��́A���݂̏��ɑ΂��āA�p�����^�Ŏw�肳�ꂽ�������Z����B
 * @param x X�ʒu
 * @param y Y�ʒu
 * @param theta ����(���W�A��)
 */
void CeilingMap::IncrementOdometry(long x, long y, double theta)
{
	m_CurOdometry.m_x += x;
	m_CurOdometry.m_y += y;
	m_CurOdometry.m_theta += theta;

	/* �}�P�W�O�x�Ɏ��܂�悤�ɒ������� */
	while (m_CurOdometry.m_theta > M_PI)
		m_CurOdometry.m_theta = -M_PI*2 + m_CurOdometry.m_theta;
	while (m_CurOdometry.m_theta < -M_PI)
		m_CurOdometry.m_theta =  M_PI*2 + m_CurOdometry.m_theta;
}

/**
 * @brief ���W�����X�V����
 * @param value ���W���
 * @note IncrementOdometry(long x, long y, double theta)�̌Ăяo�����s��
 */
void CeilingMap::IncrementOdometry(SignedOdometry_st value)
{
	IncrementOdometry(value.m_x, value.m_y, value.m_theta);
}
