#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine\olcPixelGameEngine.h"
#include "PathFinder.h"
#include<iostream>
#include<unordered_set>
#include<vector>
#include<math.h>
#include<time.h>
using namespace std;

int main()
 {
	Pathfinder P(20, 20, 0.2f,400,400,50,100);
	if (P.Construct(650, 550, 10, 10))
		P.Start();
	return 0;
}
