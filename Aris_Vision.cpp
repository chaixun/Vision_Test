#include "Aris_Vision.h"
#include <vector>
#include <string>
#include <XnCppWrapper.h>
#include "math.h"
#include <sstream>
#include <fstream>

using namespace xn;

namespace Aris
{

namespace Sensor
{

static int frameNum = 0;

struct Point3D
{
    float X;
    float Y;
    float Z;
};

void MatrixMultiple(float A[4][4], float B[4][4], float C[4][4])
{
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

float CalDet(float M[3][3])
{
    float det = M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
            + M[1][0]*(M[0][2]*M[2][1] - M[0][1]*M[2][2])
            + M[2][0]*(M[0][1]*M[1][2] - M[0][2]*M[1][1]);

    return det;
}

void InverseMatrix(float M[3][3], float InvM[3][3])
{
    double det = CalDet(M);

    InvM[0][0] = (M[1][1]*M[2][2] - M[1][2]*M[2][1]) / det;
    InvM[0][1] = (-M[1][0]*M[2][2] + M[1][2]*M[2][0]) / det;
    InvM[0][2] = (M[1][0]*M[2][1] - M[1][1]*M[2][0]) / det;
    InvM[1][0] = (-M[0][1]*M[2][2] + M[0][2]*M[2][1]) / det;
    InvM[1][1] = (M[0][0]*M[2][2] - M[0][2]*M[2][0]) / det;
    InvM[1][2] = (-M[0][0]*M[2][1] + M[0][1]*M[2][0]) / det;
    InvM[2][0] = (M[0][1]*M[1][2] - M[0][2]*M[1][1]) / det;
    InvM[2][1] = (-M[0][0]*M[1][2] + M[0][2]*M[1][0]) / det;
    InvM[2][2] = (M[0][0]*M[1][1] - M[0][1]*M[1][0]) / det;
}

void CalPlane(vector<Point3D>& cPointSet, GridMap &cgridmap)
{
    float pX[3] = {0};
    float A[3][3] = {0};
    float A_I[3][3] = {0};
    float b[3] = {0};

    for(int i = 0; i < cPointSet.size(); ++i)
    {
        A[0][0] = A[0][0] + cPointSet[i].X*cPointSet[i].X;
        A[0][1] = A[0][1] + cPointSet[i].X*cPointSet[i].Y;
        A[0][2] = A[0][2] + cPointSet[i].X;

        A[1][0] = A[0][1];
        A[1][1] = A[1][1] + cPointSet[i].Y*cPointSet[i].Y;
        A[1][2] = A[1][2] + cPointSet[i].Y;

        A[2][0] = A[0][2];
        A[2][1] = A[1][2];
        A[2][2] = cPointSet.size();

        b[0] = b[0] + cPointSet[i].X*cPointSet[i].Z;
        b[1] = b[1] + cPointSet[i].Y*cPointSet[i].Z;
        b[2] = b[2] + cPointSet[i].Z;
    }

    if(CalDet(A)==0)
    {
        float A1[2][2] = {0};
        float A1_I[2][2] = {0};
        float b1[2] = {0};
        for(int i = 0; i < cPointSet.size(); ++i)
        {
            A1[0][0] = A1[0][0] + cPointSet[i].Y*cPointSet[i].Y;
            A1[0][1] = A1[0][1] + cPointSet[i].Y;

            A1[1][0] = A1[0][1];
            A1[1][1] = i+1;

            b1[0] = b1[0] + cPointSet[i].X*cPointSet[i].Y;
            b1[1] = b1[1] + cPointSet[i].X;
        }

        float detA1 = A1[0][0]*A1[1][1] - A1[0][1]*A1[1][0];

        if(detA1 == 0)
        {
            float A2 = 0;
            float b2 = 0;
            for(int i = 0; i < cPointSet.size(); i++)
            {
                A2 = A2 + cPointSet[i].Y;
                b2 = i + 1;
            }
            pX[2] = A2/b2;

            cgridmap.planePara[0] = 0;
            cgridmap.planePara[1] = 1;
            cgridmap.planePara[2] = 0;
            cgridmap.planePara[3] = -pX[2];
        }
        else
        {
            A1_I[0][0] = A1[1][1]/detA1;
            A1_I[0][1] = -A1[1][0]/detA1;
            A1_I[1][0] = -A1[0][1]/detA1;
            A1_I[1][1] = A1[0][0]/detA1;

            pX[1] = A1_I[0][0]*b1[0] + A1_I[0][1]*b1[1];
            pX[2] = A1_I[1][0]*b1[0] + A1_I[1][1]*b1[1];

            cgridmap.planePara[0] = -1;
            cgridmap.planePara[1] = pX[1];
            cgridmap.planePara[2] = 0;
            cgridmap.planePara[3] = pX[2];
        }
    }
    else
    {
        InverseMatrix(A, A_I);
        pX[0] = A_I[0][0]*b[0] + A_I[0][1]*b[1] + A_I[0][2]*b[2];
        pX[1] = A_I[1][0]*b[0] + A_I[1][1]*b[1] + A_I[1][2]*b[2];
        pX[2] = A_I[2][0]*b[0] + A_I[2][1]*b[1] + A_I[2][2]*b[2];

        if(pX[0] > 0)
        {
            cgridmap.planePara[0] = -pX[0];
            cgridmap.planePara[1] = -pX[1];
            cgridmap.planePara[2] = 1;
            cgridmap.planePara[3] = -pX[2];
        }
        else
        {
            cgridmap.planePara[0] = pX[0];
            cgridmap.planePara[1] = pX[1];
            cgridmap.planePara[2] = -1;
            cgridmap.planePara[3] = pX[2];
        }
    }

    float distance1 = 0;
    float distance2 = sqrt(cgridmap.planePara[0]*cgridmap.planePara[0] + cgridmap.planePara[1]*cgridmap.planePara[1] + cgridmap.planePara[2]*cgridmap.planePara[2]);

    for(int i = 0; i < cPointSet.size(); i++)
    {
        distance1 = distance1 + fabs(cgridmap.planePara[0]*cPointSet[i].X + cgridmap.planePara[1]*cPointSet[i].Y + cgridmap.planePara[2]*cPointSet[i].Z + cgridmap.planePara[3]);
    }

    cgridmap.planeDegree = distance1/distance2/cPointSet.size();
    cgridmap.normalVector = acos(cgridmap.planePara[1]/distance2)/3.1415926*180;
}

void GeneratePointCloud(DepthGenerator& rDepthGen, const XnDepthPixel* pDepth, VISION_DATA &pData)
{
    DepthMetaData mDepthMD;
    rDepthGen.GetMetaData(mDepthMD);
    pData.timeStamp = mDepthMD.Timestamp();
    unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

    XnPoint3D* pDepthPointSet = new XnPoint3D[uPointNum];
    unsigned int i, j, idxshift, idx;
    for( j = 0; j < mDepthMD.FullYRes(); ++j)
    {
        idxshift = j * mDepthMD.FullXRes();

        for(i = 0; i < mDepthMD.FullXRes(); ++i)
        {
            idx = idxshift + i;
            pDepthPointSet[idx].X = i;
            pDepthPointSet[idx].Y = j;
            pDepthPointSet[idx].Z = pDepth[idx];
        }
    }

    XnPoint3D* p3DPointSet = new XnPoint3D[uPointNum];

    rDepthGen.ConvertProjectiveToRealWorld(uPointNum, pDepthPointSet, p3DPointSet);

    memcpy(pData.pointCloud, p3DPointSet, uPointNum*3*sizeof(float));

    delete[] pDepthPointSet;

    delete[] p3DPointSet;
}

void GenerateGridMap(VISION_DATA &cdata)
{
    int cGridNum[30][30] = {0};

    for(int i = 0; i < 480; i++)
    {
        for(int j = 0; j < 640; j++)
        {
            if(cdata.pointCloud[i][j][0] > -1.5 && cdata.pointCloud[i][j][0] < 1.5&&
                    cdata.pointCloud[i][j][2] > 0 && cdata.pointCloud[i][j][2] < 3)
            {
                int m = 0, n = 0;

                n = floor(cdata.pointCloud[i][j][0]/0.1) + 15;
                m = floor(cdata.pointCloud[i][j][2]/0.1);

                //Mean
                cdata.pGridMap[m][n].Y = (cdata.pGridMap[m][n].Y*cGridNum[m][n] + cdata.pointCloud[i][j][1])/(cGridNum[m][n] + 1);

                cGridNum[m][n] = cGridNum[m][n] + 1;

                cdata.pGridMap[m][n].pointNum = cGridNum[m][n];
                cdata.pGridMap[m][n].X = (n - 15) * 0.1;
                cdata.pGridMap[m][n].Z = m * 0.1;
            }
        }
    }
}

class KINECT_BASE::KINECT_BASE_STRUCT
{
    friend class KINECT_BASE;
private:
    XnStatus mStatus;
    Context mContext;
    DepthGenerator mDepthGenerator;
    XnMapOutputMode mapDepthMode;
    void CheckOpenNIError(XnStatus eResult, string sStatus);
};

void KINECT_BASE::KINECT_BASE_STRUCT::CheckOpenNIError(XnStatus eResult, string sStatus)
{
    if(eResult != XN_STATUS_OK)
    {
        cerr << sStatus << "  Error" << xnGetStatusString(eResult) << endl;
    }
    else
    {
        cout<< sStatus << " Successful " << xnGetStatusString(eResult) << endl;
    }
}

KINECT_BASE::KINECT_BASE():mKinectStruct(new KINECT_BASE_STRUCT)
{
    mKinectStruct->mStatus = XN_STATUS_OK;
}

KINECT::KINECT()
{
    ;
}

KINECT::~KINECT()
{
    ;
}

void KINECT::UpdateData(VISION_DATA &data)
{
    KINECT_BASE::UpdateData(data);

    vector<Point3D> pPointSet;

    for(int m = 7; m < 11; m++)
    {
        for(int n = 11; n < 19; n++)
        {
            pPointSet.clear();

            for(int p = 0; p < 480; p++)
            {
                for(int q = 0; q < 640; q++)
                {
                    if(data.pointCloud[p][q][0] != 0 && data.pointCloud[p][q][1] != 0&&data.pointCloud[p][q][2] != 0
                            &&floor(data.pointCloud[p][q][0]/0.1) + 15 == n&&floor(data.pointCloud[p][q][2]/0.1) == m)
                    {
                        Point3D tempPoint;
                        tempPoint.X = data.pointCloud[p][q][0];
                        tempPoint.Y = data.pointCloud[p][q][1];
                        tempPoint.Z = data.pointCloud[p][q][2];

                        pPointSet.push_back(tempPoint);
                    }
                }
            }
            CalPlane(pPointSet, data.pGridMap[m][n]);
            cout<<"pPointSet_Size:"<<pPointSet.size()<<endl;
            cout<<"Plane_Degree"<<data.pGridMap[7][22].planeDegree<<endl;
        }
    }
}

void KINECT_BASE::Initiate()
{
    mKinectStruct->mStatus = mKinectStruct->mContext.Init();
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "initialize context");
    mKinectStruct->mapDepthMode.nFPS = 30;
    mKinectStruct->mapDepthMode.nXRes = 640;
    mKinectStruct->mapDepthMode.nYRes = 480;

    mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.Create(mKinectStruct->mContext);
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Create depth Generator");
    mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.SetMapOutputMode(mKinectStruct->mapDepthMode);
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Map Mode Set");
    mKinectStruct->mStatus  = mKinectStruct->mContext.StartGeneratingAll();
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Start View Cloud");
}

void KINECT_BASE::Release()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    //mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}

KINECT_BASE::~KINECT_BASE()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    // mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}

void KINECT_BASE::UpdateData(VISION_DATA &data)
{
    mKinectStruct->mStatus = mKinectStruct->mContext.WaitAndUpdateAll();
    //mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "View Cloud");

    memset(&data, 0, sizeof(data));

    const XnDepthPixel* pDepthMap = mKinectStruct->mDepthGenerator.GetDepthMap();

    memcpy(data.depthMap, pDepthMap, 480*640*sizeof(unsigned short));

    GeneratePointCloud(mKinectStruct->mDepthGenerator, pDepthMap, data);

    /*
    float kinectAdjust[4][4] = {{-1, 0, 0, 0},
                                {0, 1, 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}};


    float kinectToRobot[4][4] = {{0.9995, 0.0134, -0.0273, 0.0224},
                                  {-0.0304, 0.5120, -0.8584, 0.2026 + 0.038},
                                  {0.0025, 0.8589, 0.5122, 0.5733},
                                  {0, 0, 0, 1}};
*/

    float kinectAdjust[4][4] = {{1, 0, 0, 0},
                                {0, 1, 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}};

    float kinectToRobot[4][4] = {{1, 0, 0, 0},
                                 {0, 1, 0, 0},
                                 {0, 0, 1, 0},
                                 {0, 0, 0, 1}};

    float robotToWorld[4][4] = {{1, 0, 0, 0},
                                {0, 1, 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}};

    float kinectToWorld[4][4] = {0}, tempMatrix[4][4] = {0};

    MatrixMultiple(kinectToRobot, kinectAdjust, tempMatrix);

    MatrixMultiple(robotToWorld, tempMatrix, kinectToWorld);


    ofstream ofs;
    stringstream out;
    out<<frameNum;
    string dataname ="../PointCloud/cloud" + out.str() + ".txt";
    ofs.open(dataname,ios::trunc);

    for (int i = 0; i < 480; i++)
    {
        for(int j = 0; j < 640; j++)
        {
            Point3D tempPoint = {0, 0, 0};

            tempPoint.X = kinectToWorld[0][0]*data.pointCloud[i][j][0] + kinectToWorld[0][1]*data.pointCloud[i][j][1]
                    + kinectToWorld[0][2]*data.pointCloud[i][j][2] + kinectToWorld[0][3];

            tempPoint.Y = kinectToWorld[1][0]*data.pointCloud[i][j][0] + kinectToWorld[1][1]*data.pointCloud[i][j][1]
                    + kinectToWorld[1][2]*data.pointCloud[i][j][2] + kinectToWorld[1][3];

            tempPoint.Z = kinectToWorld[2][0]*data.pointCloud[i][j][0] + kinectToWorld[2][1]*data.pointCloud[i][j][1]
                    + kinectToWorld[2][2]*data.pointCloud[i][j][2] + kinectToWorld[2][3];

            data.pointCloud[i][j][0] = tempPoint.X;
            data.pointCloud[i][j][1] = tempPoint.Y;
            data.pointCloud[i][j][2] = tempPoint.Z;

            ofs<<data.pointCloud[i][j][0]<<" "<<data.pointCloud[i][j][1]<<" "<<data.pointCloud[i][j][2]<<" "<<endl;
        }
    }

    frameNum++;
    ofs.close();

    GenerateGridMap(data);
}
}
}

