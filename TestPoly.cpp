//#include "TrajectoryPlanning.h"
//
//void main()
//{
//	StopRobot();
//	ResetRobot();
//	OpenMonitor();
//	vect JointConfigArray[5];
//	vect JointVelArray[5];
//	vect JointAccArray[5];
//	Sleep(2000);
//	for (int i = 0; i < 5; i++)
//	{
//		VectorInit(JointConfigArray[i]);
//		VectorInit(JointVelArray[i]);
//		VectorInit(JointAccArray[i]);
//		for (int j = 0; j < 4; j++)
//		{
//			JointVelArray[i][j] = 18.0/5;
//			if ((i == 0) && (j == 2))
//			{
//				JointConfigArray[i][j] += -200;
//
//			}
//			else
//			{
//				JointConfigArray[i][j] += 18.0/5;
//			}
//
//		}
//	}
//	TraExec(JointConfigArray, JointVelArray, JointAccArray, 0.2, 5);
//
//}