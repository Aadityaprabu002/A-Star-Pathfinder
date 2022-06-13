#include "olcPixelGameEngine\olcPixelGameEngine.h"
#include<iostream>
#include<unordered_set>
#include<vector>
#include<math.h>
#include<time.h>
using namespace std;

class Pathfinder : public olc::PixelGameEngine
{
private:

	struct node {
		int x;
		int y;
		double h;
		double g;
		double f;
		bool obstacle;
		bool visited;
		node* parent;

		vector<node*> neighbours;
		vector<node*> neighbours_wd;//Neighbours Without Diagonals
		node(int x = 0, int y = 0) {
			this->obstacle = false;
			this->visited = false;
			this->parent = NULL;
			this->h = 0.0;
			this->g = 0.0;
			this->f = 0.0;
			this->x = x;
			this->y = y;
		}
		bool operator == (const node& n) const {
			if (this->x == n.x and this->y == n.y) return true;
			return false;
		}
	};


	int ofs_x;//Offset x
	int ofs_y;//Offset y
	int GWidth;//Graph Width
	int GHeight;//Graph Height
	bool PathFound;
	float PathLength;
	bool PathFound_wd;
	float PathLength_wd;
	node** Grid;
	node* End;
	node* StartNode;
	int GridRows;
	int GridCols;
	float ObstaclePopulation;
	vector<node*> OpenSet;
	vector<node*> ClosedSet;
	vector<node*> SolvedPath;
	vector<node*> SolvedPath_wd;
	double Heuristics(node* Neighbour, node* End);
	bool OnUserCreate() override;
	void FindPath(vector<node*> &SP, float &PL, bool &PF,int choice);
	//choice Values : 1 - A* with Diagonals, 2 - A* Without Diagonal
	int flag, flag1;//For Menu Buttons
	void PrintPath(vector<node*>& SP, float& PL, olc::Pixel Color,float LW);//LW-Line Width
	bool OnUserUpdate(float fElapsedTime) override;
	void PrintTitleBar();
	void PrintMenu();
public:
	Pathfinder(int, int, float,int,int,int,int);
	~Pathfinder();
};


