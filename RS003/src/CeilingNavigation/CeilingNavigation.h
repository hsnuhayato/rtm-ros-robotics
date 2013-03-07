// -*- C++ -*-
/*!
 * @file  CeilingNavigation.h
 * @brief CeilingNavigation
 * @date  $Date$
 *
 * $Id$
 */

#ifndef CEILINGNAVIGATION_H
#define CEILINGNAVIGATION_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "std_hdr.h"

#include "intellirobotStub.h"

#include "BlockMatching.h"
#include "ImageData.h"
#include "CeilingMap.h"

using namespace RTC;

/**
 *@brief �V��i�r�Q�[�V����RTC���C���N���X
 */
class CeilingNavigation
  : public RTC::DataFlowComponentBase
{
 public:
  CeilingNavigation(RTC::Manager* manager);
  ~CeilingNavigation();
 
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
	std::string m_NavigationMap;		/* �V��摜�t�@�C����						*/
	unsigned short m_BlockResolution;	/* �摜��]����\							*/
	unsigned short m_BlockSize;			/* �摜�k���T�C�Y							*/
	double m_BlockCoefficient;			/* �摜�k���␳�l							*/
	unsigned short m_SearchScope;		/* �}�b�`���O�T���͈�						*/
	unsigned short m_BlackWhiteValue;	/* 2�l��臒l								*/
	long m_Center_X;					/* �摜��]���S���W(X)						*/
	long m_Center_Y;					/* �摜��]���S���W(Y)						*/
	std::string m_OfflineImage;			/* �I�t���C���C���[�W�t�@�C���i�[�t�H���_�� */
	double m_RealMapHeight;
	double m_RealMapWidth;
	double m_VirtualMapHeight;
	double m_VirtualMapWidth;

	unsigned long x_pos;
	unsigned long y_pos;
	double theta;

	double mPerPix;//�摜1�s�N�Z��������̎����E�̒���[m]
	//���{�b�g���W�n�́C�O����x,����y�Ƃ���D
	double camX; //���{�b�g���W�n�ɂ�����J������x���W
	double camY; //���{�b�g���W�n�ɂ�����J������y���W
	//�p�x�ɂ��ẮC�摜�̏���������{�b�g���Wx���������Ɉ�v����悤�ɔz�u����Ƃ������ƂŌŒ肷��D

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedOctetSeq m_CameraData;
  InPort<TimedOctetSeq> m_CameraDataIn;
 
	IIS::TimedPose2D m_LocalizedPosition;
	InPort<IIS::TimedPose2D> m_LocalizedPositionIn; 

  IIS::TimedPose2D m_CeilingPosition;
  OutPort<IIS::TimedPose2D> m_CeilingPositionOut;
  
 private:
  int dummy;
  private:
	  BlockMatching	m_BlockMat;
	  CeilingMap	m_CeilingMap;

};


extern "C"
{
  void CeilingNavigationInit(RTC::Manager* manager);
};

#endif // CEILINGNAVIGATION_H
