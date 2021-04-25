#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<fstream>
#include <sstream>
#include <iomanip>

#include<random>
#include<cmath>  
#include<cfloat>  
 
using namespace std;
using namespace cv;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void CallMatchNum(Mat img_1, Mat img_2, int &numAll, int &numMatched)
{
    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
 
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );
	
	numAll = keypoints_1.size() + keypoints_2.size();
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );


    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;



    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

	numMatched = good_matches.size();
}

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
	cout << "Loading images for sequence " << endl << "...";
	vector<string> vstrImageFilenames;
	vector<double> vTimestampsCam;
	string pathSeq(argv[1]);          //数据集地址
	string pathTimeStamps(argv[2]);   //时间文件（图像名称）

	string pathCam0 = pathSeq + "/mav0/cam0/data";

	LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames, vTimestampsCam);
	cout << "LOADED!" << endl;
    //-- 读取图像
    double percent = 0.0;
	std::uniform_real_distribution<double> dis(0, std::nextafter(1,DBL_MAX));
	//定义随机数种子
	std::random_device seed;
	//定义默认随机数生成器
	std::default_random_engine dre{ seed() };
	
    double lost = 0.2;
	for(int i = 0; i < vstrImageFilenames.size() - 1; i++)
	{   
        double temp = dis(dre);
		if(temp <= lost)
		{
			continue;
		}
		//cout << "i:" <<i<<"  rand: "<<setprecision(4)<<temp << endl;
		for(int j= i+1; j < vstrImageFilenames.size(); j++)
		{   
			double temp2 = dis(dre);
			if(temp2 <= lost)
			{
				continue;
			}
		//	cout << "j: " <<j<<"  rand:"<<setprecision(4)<<temp2 << endl;			
			Mat img_1 = imread (vstrImageFilenames[i], CV_LOAD_IMAGE_COLOR );
			Mat img_2 = imread (vstrImageFilenames[j], CV_LOAD_IMAGE_COLOR );
			int numAll =1;
			int numMatched = 1; 
			CallMatchNum(img_1, img_2, numAll, numMatched);
			if(numAll == 0 || numMatched == 0)
			{
				cout << "error occurred numAll:" <<numAll << ", numMatched:" <<numMatched <<endl;
				continue;
			}
			
			
			percent = ((double)numMatched/(double)numAll + i*percent)/(i+1.0);  //+ i*percent)/(i+1)
		
			if(i%100 == 0)
			{
				cout << numMatched<<","<<numAll<<","<<fixed << setprecision(4)<<percent << endl;
			}
			break;
		}

	}
	cout << "end" << endl;
    cout <<cout.precision(15)<< percent << endl;
    return 0;
}

