#include "olcPixelGameEngine\olcPixelGameEngine.h"
#include<iostream>
#include<vector>
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
		node* parent;
		vector<node*> neighbours;
		vector<node*> neighbours_wd;//Neighbours Without Diagonals
		node(int x = 0, int y = 0) {
			this->obstacle = false;
			
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
	int flag, flag1;//For Menu Buttons


	bool OnUserUpdate(float fElapsedTime) override;

	bool OnUserCreate() override;


	/*
	Heuristics is a function used to guess the minimum distance between two nodes. 
	It takes in one neighbour and the end node and returns the tentative distance between them.
	*/
	double Heuristics(node* Neighbour, node* End);

	/*
	Find Path Function implements the A* Algorithm.
	It takes in parameters SolvedPath(SP), PathLength(PL), and PathFound(PF) and choice of path.
	Choice Values : 1 for Path with Diagonals, 2 for Path Without Diagonal

	*/
	void FindPath(vector<node*> &SP, float &PL, bool &PF,int choice); 
	
	/*
	PrintTitleBar Function is used to display title bar of the graphical interface.
	*/
	void PrintTitleBar();

	/*
	PrintMenu function is used to display the Menu system of the graphical interface.
	*/
	void PrintMenu();
	
	/*
	PrintPath function is used to Print the path in the Graph/Grid.
	It takes in parameters SolvedPath(SP), PathLength(PL), Color of Pixel(Color) and Line Width(LW).
	
	*/
	void PrintPath(vector<node*>& SP, float& PL, olc::Pixel Color,float LW); 
	
	
public:
	/*
	This is the Pathfinder class constructor.
	It takes in parameters: Number of grid rows(GridRows), Number of Grid columns (GridCols), ObstaclePopulation,
	Graph Width (GWidth),Graph Height(GHeight),Offset X(ofs_x), Offset Y(ofs_y).
	*/
	Pathfinder(int GridRows, int GridCols , float ObstaclePopulation, int GWidth , int GHeight, int ofs_x , int ofs_y);
	~Pathfinder();
};


