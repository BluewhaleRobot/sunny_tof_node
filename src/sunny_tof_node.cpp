#include<sunny_tof_node.h>
#include <boost/thread.hpp>

//以下部分可封装到类中
ros::Publisher tof_pub;
ros::Publisher scan_pub;
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ> pub_cloud;
sensor_msgs::PointCloud2 rosCloud;
sensor_msgs::LaserScan rosScan;

bool bRunning = true;
boost::mutex mMutex;

/*点云转激光部分参数*/
float mAngleMin = -1.08960903;                //激光最小角度
float mAngleMax = 1.08960903;                 //激光最大角度
float mAngleIncrement = 0.00972865;           //激光分辨率
float mRangeMin = 0.1;                        //最近量程
float mRangeMax = 7.0;                        //最远量程
float mScanTime = 0.05;                       //每帧扫描用时

//点云转激光
void CloudToLaserScan(const pcl::PointCloud<pcl::PointXYZ> cloud_in,   //输入点云
  sensor_msgs::LaserScan& scan_out)                //输出转换好的ros中激光类型
  {
    /*激光参数部分*/
    scan_out.angle_min = mAngleMin;
    scan_out.angle_max = mAngleMax;
    scan_out.angle_increment = mAngleIncrement;
    scan_out.range_min = mRangeMin;
    scan_out.range_max = mRangeMax;
    scan_out.scan_time = mScanTime;

    /*激光数据部分*/
    uint32_t ranges_size = std::ceil((scan_out.angle_max - scan_out.angle_min) / scan_out.angle_increment); //计算有多少条激光束，向上取整
    scan_out.ranges.assign(ranges_size,std::numeric_limits<float>::infinity());         //激光数据的初始值设置为无穷大

    float now_range,now_angle;                                                          //用于存储当前点的距离和方向
    for(size_t i=0; i<cloud_in.points.size(); ++i)                                     //遍历所有点，转换激光
    {
      now_range = hypot(cloud_in.points[i].z, cloud_in.points[i].x);
      now_angle = atan2(cloud_in.points[i].x, cloud_in.points[i].z);

      //计算索引
      int now_index = (now_angle - scan_out.angle_min) / scan_out.angle_increment;

      if(now_range < scan_out.ranges[now_index])
      {
        scan_out.ranges[now_index] = now_range;
      }
    }
  }

  template <class T>
  T FindMaxValue(T* pData, const int nCnt)
  {
    T max = pData[0];

    for (int i = 0; i < nCnt; i++)
    {
      if (max < pData[i])
      {
        max = pData[i];
      }
    }

    return max;
  }

  template <class T>
  T FindMinValue(T* pData, const int nCnt)
  {
    T min = pData[0];

    for (int i = 0; i < nCnt; i++)
    {
      if (min > pData[i])
      {
        min = pData[i];
      }
    }

    return min;
  }

  class CGrayConvert
  {
    CGrayConvert() {};
    ~CGrayConvert() {};

  public:
    static bool Gray_2_Bgr32(const GRAY_FORMAT format, void* pGray, const int width, const int height, unsigned char* pBgr32)
    {
      bool retVal = false;

      //long tick1 = GetTickCount();

      switch (format)
      {
        case GRAY_FORMAT_UINT8: retVal = Gray_2_Bgr32((unsigned char*)pGray, width, height, pBgr32); break;
        case GRAY_FORMAT_UINT16: retVal = Gray_2_Bgr32((unsigned short*)pGray, width, height, pBgr32); break;
        case GRAY_FORMAT_FLOAT: retVal = Gray_2_Bgr32((float*)pGray, width, height, pBgr32); break;
        case GRAY_FORMAT_BGRD:
        {
          memcpy(pBgr32, pGray, width* height * 4);
          retVal = true;
          break;
        }
        default: break;
      }

      return retVal;
    }

  private:
    static bool Gray_2_Bgr32(unsigned char* pGray, const int width, const int height, unsigned char* pBgr32)
    {
      const int pixel_cnt = width*height;
      int offset_dest = 0;
      for (int i = 0; i < pixel_cnt; i++)
      {
        pBgr32[offset_dest + 0] = pGray[i];
        pBgr32[offset_dest + 1] = pGray[i];
        pBgr32[offset_dest + 2] = pGray[i];
        pBgr32[offset_dest + 3] = 0;

        offset_dest += 4;
      }

      return true;
    }
    static bool Gray_2_Bgr32(unsigned short* pGray, const int width, const int height, unsigned char* pBgr32)
    {
      const int pixel_cnt = width*height;
      const unsigned short max = FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

      if (0 >= max)
      {
        memset(pBgr32, 0, pixel_cnt * 4);
        return true;
      }

      const double K = (255 * 1.0 / max);//最大值是255的多少倍

      int offset_dest = 0;
      for (int i = 0; i < pixel_cnt; i++)
      {
        const unsigned char tmp = (unsigned char)(pGray[i] * K);
        pBgr32[offset_dest + 0] = tmp;
        pBgr32[offset_dest + 1] = tmp;
        pBgr32[offset_dest + 2] = tmp;
        pBgr32[offset_dest + 3] = 0;

        offset_dest += 4;
      }

      return true;
    }
    static bool Gray_2_Bgr32(float* pGray, const int width, const int height, unsigned char* pBgr32)
    {
      const int pixel_cnt = width*height;
      const float max = FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

      if (0.001 >= max)//0值用黑色表示
      {
        memset(pBgr32, 0, pixel_cnt * 4);
        return true;
      }

      const double K = (255 * 1.0 / max);//最大值是255的多少倍

      int offset_dest = 0;
      for (int i = 0; i < pixel_cnt; i++)
      {
        unsigned char tmp = 0;//0值用黑色表示
        if (0.001 < pGray[i])
        {
          tmp = (unsigned char)(pGray[i] * K);
        }
        pBgr32[offset_dest + 0] = tmp;
        pBgr32[offset_dest + 1] = tmp;
        pBgr32[offset_dest + 2] = tmp;
        pBgr32[offset_dest + 3] = 0;

        offset_dest += 4;
      }

      return true;
    }

  };

  static void CallBackTofDeviceStatus(TOFDEV_STATUS tofDevStatus, void *pUserData)
  {
    printf("device status: %d.\n", tofDevStatus);

  }

  void fnTofStream(TofFrameData *tofFrameData, void* pUserData)
  {
    //发布数据,限制为10hz
    static ros::WallTime last_ordertime = ros::WallTime::now();;
    ros::WallDuration t_diff = ros::WallTime::now() - last_ordertime;
    if(t_diff.toSec()<0.08) return;

    pub_cloud.points.clear();				//先清空旧数据
    pcl::PointXYZ point;

    for(UINT32 i=0;i < tofFrameData->frameWidth*tofFrameData->frameHeight; ++i)
    {
      point.x = tofFrameData->pPointData[i].x;
      point.y = tofFrameData->pPointData[i].y;
      point.z = tofFrameData->pPointData[i].z;
      pub_cloud.points.push_back(point);
    }

    pcl::toROSMsg(pub_cloud, rosCloud);
    rosCloud.header.frame_id = "tof_pointcloud_link";
    rosCloud.header.stamp = ros::Time::now();
    tof_pub.publish(rosCloud);

    /*点云转激光*/
    CloudToLaserScan(pub_cloud,rosScan);
    rosScan.header.frame_id = "laser_tof";
    rosScan.header.stamp = ros::Time::now();
    scan_pub.publish(rosScan);
  }

  static void PrintDevInfo(TofDeviceInfo *pTofDeviceInfo)
  {
    printf("Dev Info:==================================\n");
    printf(">>  szDevName=%s.\n", pTofDeviceInfo->szDevName);
    printf(">>  szDevId=%s.\n", pTofDeviceInfo->szDevId);
    printf(">>  szFirmwareVersion=%s.\n", pTofDeviceInfo->szFirmwareVersion);
    printf("Dev Info==================================\n\n");
  }

  void CleanerTof::StartCleaner()
  {
    //对第一台设备操作
    HTOFD hTofD = TOFD_OpenDevice(&this->mDevsDescList[0], CallBackTofDeviceStatus, NULL);	//进入回调函数，打开设备
    if (NULL == hTofD)													//处理打开失败的情况
    {
      printf("Open Tof Device failed\n");
      getchar();
      return;
    }

    TofDeviceInfo struCaps;
    memset(&struCaps, 0, sizeof(struCaps));
    TOFRET retVal = TOFD_GetDeviceInfo(hTofD, &struCaps);
    PrintDevInfo(&struCaps);

    for (int i = 0; i<32; i++)
    {
      int tofMode = 1 << i;
      if ((struCaps.supportedTOFMode & tofMode) == 0)
      {
        continue;
      }

      TOFD_StopTofStream(hTofD);
      TOFRET ret = TOFD_StartTofStream(hTofD, (TOF_MODE)tofMode, fnTofStream, this);

      if (TOFRET_SUCCESS != ret)
      {
        printf("***************test tof mode 0x%08x, [ FAILED ]!!!!!!!!!!!!!!!!!!!!!!!!!!! %d\n\n",tofMode, i);
        TOFD_StopTofStream(hTofD);
        boost::mutex::scoped_lock lock(mMutex);
        bRunning = false;
        break;
        //getchar();//测试失败，则卡住
      }
      else
      {
        printf("***************test tof mode 0x%08x, [ success ]!!!!!!!!!!!!!!!!!!!!!!!!!!! %d\n\n",tofMode,i);
      }

      //(没有主动要求退出的情况下)一直出流
      while (bRunning && ros::ok())
      {
        ros::Duration(5).sleep();
      }

      while (ros::ok())
      {
        ros::Duration(1).sleep();
      }

      TOFD_StopTofStream(hTofD);
      //等待线程退出
      {
        boost::mutex::scoped_lock lock(mMutex);
        bRunning = false;
      }

    }

    TOFD_CloseDevice(hTofD);															//关闭设备
    printf("-----Test Tof Streaming [PASS]-------\n");
    //delete &struCaps;
  }

  //线程管理，启动所有线程
  void CleanerTof::StartAllThread()
  {
    //进入回调函数,执行下面取流操作
    std::thread  cleaner(&CleanerTof::StartCleaner,this);
    cleaner.detach();

    //设置线程停止条件
    bool cleaner_Running =true;
    while(ros::ok()&&cleaner_Running)
    {
      {
        boost::mutex::scoped_lock lock(mMutex);
        cleaner_Running = bRunning;
      }
      ros::Duration(1).sleep();
    }
  }

  //默认构造
  CleanerTof::CleanerTof()
  {
  }

  int main(int argc, char** argv)
  {
    /* 初始化节点 */
    ros::init(argc, argv, "sunny_tof_node");

    //构造线性tof发布对象
    ros::NodeHandle n;
    tof_pub = n.advertise<sensor_msgs::PointCloud2>("/tof_pointcloud",5);
    scan_pub = n.advertise<sensor_msgs::LaserScan>("/tof_scan",5);

    printf("*********************start test device*************************\n");
    TofDevInitParam struInitParam;
    memset(&struInitParam, 0, sizeof(struInitParam));
    strncpy(struInitParam.szDepthCalcCfgFileDir, "./parameter", sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
    TOFD_Init(&struInitParam);
    printf("SDK Version: %s.\n", TOFD_GetSDKVersion());     //输出SDK版本
    TofDeviceDescriptor* pDevsDescList = NULL;
    UINT32  dev_num = 0;
    TOFD_SearchDevice(&pDevsDescList, &dev_num);			//寻找设备，输入为设备描述以及设备个数的引用

    /* 实例化对象 */
    CleanerTof*  SubAndPubTof = new CleanerTof;

    /* 初始化工作 */
    SubAndPubTof->mDevsDescList = pDevsDescList;

    /* 启动线程 */
    if(dev_num == 1)
    {
      SubAndPubTof->StartAllThread();
    }else
    {
      printf("can not find tof defvice!\n");
    }

    printf("*********************stop test device*********************\n");
    TOFD_Uninit();

    return 0;
  }
