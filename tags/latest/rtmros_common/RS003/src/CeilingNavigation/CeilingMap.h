#pragma once

#include "intellirobotStub.h"
#include "std_hdr.h"
#include "ImageData.h"

typedef struct _tag_CeilingOdometry
{
	unsigned long	m_x;
	unsigned long	m_y;
	double			m_theta;
} CeilingOdometry_st, *CeilingOdometry_pst;

typedef struct _tag_SignedOdometry
{
	long	m_x;
	long	m_y;
	double	m_theta;
} SignedOdometry_st, *SignedOdometry_pst;

/**
 *@brief �V��摜�Ǘ��N���X
 */
class CeilingMap : public ImageData
{
public:
	CeilingMap(void);
	~CeilingMap(void);

	void ReadNavigationMap(std::string map_name);
	void SetLocation(unsigned long x, unsigned long y, double theta);
	void SetLocation(CeilingOdometry_st value);
	CeilingOdometry_st	GetLocation();
	void IncrementOdometry(long x, long y, double theta);
	void IncrementOdometry(SignedOdometry_st value);

	/**
	 * @brief �}�b�`���O��̑Ώۂ̌������擾����
	 * @return �Ώۂ̌���(���W�A��)
	 */
	double	GetTheta()	{ return m_CurOdometry.m_theta; }

	/**
	 * @brief �}�b�`���O��̑Ώۂ�X�ʒu���擾����
	 * @return �Ώۂ�X�ʒu
	 */
	unsigned long	GetPosX()	{ return m_CurOdometry.m_x; }

	/**
	 * @brief �}�b�`���O��̑Ώۂ�Y�ʒu���擾����
	 * @return �Ώۂ�Y�ʒu
	 */
	unsigned long	GetPosY()	{ return m_CurOdometry.m_y; }

private:
	CeilingOdometry_st	m_CurOdometry;	/* �Ώۂ̈ʒu���	*/
};
