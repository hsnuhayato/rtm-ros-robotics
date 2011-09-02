#pragma once

#include "std_hdr.h"
#include "CeilingMap.h"
#include "ImageData.h"

/**
 *@brief �u���b�N�}�b�`���O����N���X
 */
class BlockMatching
{
public:
	BlockMatching(void);
	~BlockMatching(void);

	void CreateBlockMap(const long center_x,
						const long center_y,
						const unsigned long image_width,
						const unsigned long image_height,
						const unsigned short block_size,
						const double Coefficient,
						const unsigned short Resolution,
						unsigned short BlackWhiteValue);
	void MapTracking(ImageData &CeilingMap, ImageData &CurPos);
	void MapTracking(CeilingMap &CeilingMap, ImageData &CurImage);
	int	SumOfAbsoluteDifference(ImageData &src, ImageData &data, unsigned long offset_x = 0, unsigned long offset_y = 0);
	SignedOdometry_st Matching(CeilingMap &CeilingMap, ImageData &CurImage, int reach);
	void Packing(ImageData &org, ImageData &packData, double theta);

private:
	void CreateBlockTable();
	void DeleteBlockTable();

	long				***m_BlockTable;	/* �摜�̉�]�E�k���e�[�u��		*/
	unsigned short		m_LatticeSize;		/* �摜�̍ŏI�T�C�Y				*/
	int					m_LatticeSize_Z;	/* �摜��360��]�̕���\		*/
	int					m_Resolution;		/* �摜��360��]�̕���\		*/
	unsigned short		m_BlackWhiteValue;	/* 2�l��臒l					*/
};
