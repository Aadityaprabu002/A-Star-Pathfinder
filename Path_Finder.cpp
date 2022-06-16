
#include "olcPixelGameEngine\olcPixelGameEngine.h"
#include "PathFinder.h"
#include<iostream>
#include<vector>
#include<math.h>
#include<time.h>
#include<iterator>
#include<stdlib.h>
using namespace std;

//Amaraja Part
Pathfinder::Pathfinder(int GridRows = 0, int GridCols = 0, float ObstaclePopulation = 0.4, int GWidth = 400, int GHeight = 400, int ofs_x = 0, int ofs_y = 100)
{
	this->sAppName = "A* PATH FINDING ALGORITHM!";
	this->GridRows = GridRows;
	this->GridCols = GridCols;
	this->ObstaclePopulation = ObstaclePopulation;
	this->GWidth = GWidth;
	this->GHeight = GHeight;
	this->ofs_x = ofs_x;
	this->ofs_y = ofs_y;
	this->PathFound = false;
	this->flag = 0; // used for menu gui
	this->flag1 = 0; // used for menu gui

}

Pathfinder::~Pathfinder() {
	for (int i = 0; i < GridRows; i++) {
		delete[] Grid[i];
	}
	delete[] Grid;
}

bool Pathfinder::OnUserCreate() {

	Grid = new node * [GridRows];
	for (int i = 0; i < GridRows; i++) {
		Grid[i] = new node[GridCols];
	}
	StartNode = &Grid[0][0];
	End = &Grid[GridRows - 1][GridCols - 1];

	for (int i = 0; i < GridRows; i++) {
		for (int j = 0; j < GridCols; j++) {
			if (i > 0) {//LEFT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i - 1][j]);
				Grid[i][j].neighbours_wd.push_back(&Grid[i - 1][j]);

			}
			if (i < GridRows - 1) {//RIGHT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i + 1][j]);
				Grid[i][j].neighbours_wd.push_back(&Grid[i + 1][j]);
			}
			if (j > 0) {//UPPER NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i][j - 1]);
				Grid[i][j].neighbours_wd.push_back(&Grid[i][j - 1]);
			}
			if (j < GridCols - 1) {//LOWER NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i][j + 1]);
				Grid[i][j].neighbours_wd.push_back(&Grid[i][j + 1]);
			}
			// ADDING DIAGONAL
			if (i > 0 and j > 0) {//UPPERLEFT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i - 1][j - 1]);
			}
			if (i < GridRows - 1 and j < GridCols - 1) {//LOWERRIGHT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i + 1][j + 1]);
			}
			if (i > 0 and j < GridCols - 1) {//LOWERLEFT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i - 1][j + 1]);
			}
			if (j > 0 and i < GridRows - 1) {//UPPERRIGHT NEIGHBOUR
				Grid[i][j].neighbours.push_back(&Grid[i + 1][j - 1]);
			}

			Grid[i][j].x = i;
			Grid[i][j].y = j;

			float RandomPopulation = (float)rand() / RAND_MAX; // random value to be gen
			cout << RandomPopulation << " " << ObstaclePopulation << endl;
			if (RandomPopulation < ObstaclePopulation and &Grid[i][j] != StartNode and &Grid[i][j] != End) {
				Grid[i][j].obstacle = true;
			}
		}
	}
	FindPath(SolvedPath, PathLength, PathFound, 1);
	FindPath(SolvedPath_wd, PathLength_wd, PathFound_wd, 2);
	return true;

}


//Aaditya  Part

double Pathfinder::Heuristics(node* Neighbour, node* End) {
		//return abs(Neighbour->x - End->x) + abs(Neighbour->y - End->y);
		return sqrt(pow(Neighbour->x - End->x, 2) + pow(Neighbour->y - End->y, 2));
}

void Pathfinder::FindPath(vector<node*>& SP, float& PL, bool& PF, int choice) {
	SP.clear();
	OpenSet.clear();
	ClosedSet.clear();
	OpenSet.push_back(StartNode);
	PF = false;

	while (!OpenSet.empty()) {

		int MinIndex = 0;

		for (int i = 0; i < OpenSet.size(); i++) {
			if (OpenSet[i]->f < OpenSet[MinIndex]->f) {
				MinIndex = i;
			}
		}

		node* Current = OpenSet[MinIndex];

		if (Current == End) {

			node* Temp = Current;
			while (Temp != StartNode) {
				SP.push_back(Temp);
				Temp = Temp->parent;
			}
			SP.push_back(Temp);
			cout << "Found path! " << endl;
			PF = true;

			break;
		}


		for (auto itr = OpenSet.begin(); itr != OpenSet.end(); itr++) {
			if (*itr == Current) {
				OpenSet.erase(itr);
				break;
			}
		}

		ClosedSet.push_back(Current);

		vector<node*>::iterator i, j;

		if (choice == 1) {
			i = Current->neighbours.begin();
			j = Current->neighbours.end();
		}
		else if (choice == 2) {
			i = Current->neighbours_wd.begin();
			j = Current->neighbours_wd.end();
		}

		for (i; i != j; i++) {
			node* Neighbour = *i;
			auto itr = find(ClosedSet.begin(), ClosedSet.end(), Neighbour);
			if (itr != ClosedSet.end()) {
				continue;
			}
			if (Neighbour->obstacle) {
				continue;
			}
			double Tentative_gScore = Current->g + 1.0;

			itr = find(OpenSet.begin(), OpenSet.end(), Neighbour);
			bool flag = false;
			if (itr != OpenSet.end()) {
				if (Tentative_gScore < Neighbour->g) {
					Neighbour->g = Tentative_gScore;
					flag = true;
				}
			}
			else {
				Neighbour->g = Tentative_gScore;
				OpenSet.push_back(Neighbour);
				flag = true;
			}
			if (flag == true) {
				Neighbour->h = Heuristics(Neighbour, End);
				Neighbour->f = Neighbour->g + Neighbour->h;
				Neighbour->parent = Current;
			}

		}
	
	}
	return;

}

bool Pathfinder::OnUserUpdate(float fElapsedTime)
{
		PrintTitleBar();
		PrintMenu();
		int NodeWidth = (GWidth/ GridCols);
		int NodeHeight = (GHeight / GridRows);
		
		int Correction = 1;
		int mx = GetMouseX() - ofs_x;
		int my = GetMouseY() - ofs_y;
		if (GetMouse(0).bPressed) {

			int x = GetMouseX() - ofs_x;
			int y = GetMouseY() - ofs_y;
			if (0< x && x < GWidth - 1 && y > 0 && y < GHeight - 1) 
			{
				int x1 = x / NodeWidth;
				int y1 = y / NodeHeight;
				if (&Grid[x1][y1] != StartNode and &Grid[x1][y1] != End)
				{
					Grid[x1][y1].obstacle = !Grid[x1][y1].obstacle;
					FindPath(SolvedPath,PathLength,PathFound,1);
					FindPath(SolvedPath_wd, PathLength_wd, PathFound_wd, 2);
				}
			}
		}

		if (flag1 == 1) {
			PrintPath(SolvedPath, PathLength, olc::CYAN, 3);
		}
		else if (flag1 == 2) {
			PrintPath(SolvedPath_wd, PathLength_wd, olc::ORANGE, 3);
		}
		else {
			PrintPath(SolvedPath, PathLength, olc::CYAN, 4);
			PrintPath(SolvedPath_wd, PathLength_wd, olc::ORANGE, 1.5);
		}

		DrawStringDecal(olc::vf2d((float)(2 * ofs_x),(float) (5 * ofs_y / 4 + GHeight)), "CREDITS: MADE USING OneLoneCoder.com - Pixel Game Engine", olc::WHITE, olc::vf2d(1.0f, 1.0f));

		return true;
} 


//Saisathish Part

void Pathfinder::PrintPath(vector<node*>& SP, float& PL, olc::Pixel Color,float LW) {
	int NodeWidth = (GWidth / GridCols);
	int NodeHeight = (GHeight / GridRows);

	int Correction = 1;
	
	PL = 0.0;

	for (int i = 0; i < GridRows; i++) {
		for (int j = 0; j < GridCols; j++)
		{
			//FOR THE GRAPH MATRIX
			FillRect(ofs_x + (i * NodeWidth), ofs_y + (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::WHITE);
					
			if (&Grid[i][j] == StartNode) {
				FillRect(ofs_x + (i * NodeWidth), ofs_y + (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::GREEN);
			}
			if (&Grid[i][j] == End) {
				FillRect(ofs_x + (i * NodeWidth), ofs_y + (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::RED);
			}
			if (Grid[i][j].obstacle) {
				FillRect(ofs_x + (i * NodeWidth), ofs_y + (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::VERY_DARK_GREY);
			}

			DrawRect(ofs_x + (i * NodeWidth), ofs_y + (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::Pixel(0, 0, 0));



			//FOR THE PATH
			auto itr = find(SP.begin(), SP.end(), &Grid[i][j]);
			if (itr != SP.end()) {
				auto itr1 = itr + 1;
				int HalfNW = NodeWidth / 2;
				int HalfNH = NodeHeight / 2;

				if (itr1 != SP.end())
				{
					int x1 = ofs_x + (*itr)->x * NodeWidth + HalfNW;
					int y1 = ofs_y + (*itr)->y * NodeHeight + HalfNH;
					int x2 = ofs_x + (*itr1)->x * NodeWidth + HalfNW;
					int y2 = ofs_y + (*itr1)->y * NodeHeight + HalfNH;

					DrawLineDecal(olc::vf2d((float)x1, (float)y1), olc::vf2d((float)x2, (float)y2), olc::WHITE);
					for (float C = 0.1f; C < LW/2; C =(float)(C + 0.1)) {
						DrawLineDecal(olc::vf2d((float)(x1 + C), (float)(y1 + C)), olc::vf2d((float)(x2 + C), (float)(y2 + C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 - C), (float)(y1 - C)), olc::vf2d((float)(x2 - C), (float)(y2 - C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 + C), (float)(y1 + C)), olc::vf2d((float)(x2 - C), (float)(y2 - C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 - C), (float)(y1 - C)), olc::vf2d((float)(x2 + C), (float)(y2 + C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 + C), (float)(y1 - C)), olc::vf2d((float)(x2 + C), (float)(y2 - C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 - C), (float)(y1 + C)), olc::vf2d((float)(x2 - C), (float)(y2 + C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 + C), (float)(y1 - C)), olc::vf2d((float)(x2 - C), (float)(y2 + C)), Color);
						DrawLineDecal(olc::vf2d((float)(x1 - C), (float)(y1 + C)), olc::vf2d((float)(x2 + C), (float)(y2 - C)), Color);
					}

					if (x1 == x2 or y1 == y2) {
						PL += 1.0;
					}
					else {
						PL += (float)sqrt(2.0);
					}

					int C = 5*Correction;

					

				}

			}

		}
	}


}

void Pathfinder::PrintTitleBar() {
	Clear(olc::DARK_BLUE);
	DrawStringDecal(olc::vf2d((float)(ofs_x), (float)(ofs_y / 6)), "SIMULATION OF A* ALGORITHM", olc::WHITE, olc::vf2d(2.5f, 2.5f));
	DrawStringDecal(olc::vf2d((float)(ofs_x), (float)(ofs_y / 2)), "PROJECT BY : 1. AADITYA PRABHU K - 2020115001 \n\n\t\t\t 2. AMARAJA VIJAYKUMAR - 2020115008 \n\n\t\t\t 3. SAISATHISH KARTHIKEYAN - 2020115071", olc::WHITE, olc::vf2d(1.0f, 1.0f));

}

void Pathfinder::PrintMenu() {
	int C = 1;

	DrawRect(GWidth + 3 * ofs_x / 2, ofs_y, ofs_x * 2, ofs_y / 2, olc::DARK_GREEN);
	DrawRect(GWidth + 3 * ofs_x / 2, 2 * ofs_y, ofs_x * 2, ofs_y / 2, olc::DARK_RED);
	
	if (flag == 1) {
		FillRect(GWidth + 3 * ofs_x / 2 + 1, ofs_y + 1, ofs_x * 2 - 1, ofs_y / 2 - 1, olc::DARK_GREEN);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5),(float) ( ofs_y + 5)), "SELECT POINT\n\nTO CHANGE\n\nTO SOURCE", olc::BLACK, olc::vf2d(1.0f, 1.0f));
		FillRect(GWidth + 3 * ofs_x / 2 + C, 2 * ofs_y + C, ofs_x * 2 - C, ofs_y / 2 - C, olc::RED);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C), (float)(2 * ofs_y + 5 * C)), "CLICK HERE\n\nTO CHANGE\n\nDESTINATION", olc::BLACK, olc::vf2d(1.0f, 1.0f));

	}
	else if (flag == 2) {
		FillRect(GWidth + 3 * ofs_x / 2 + C, ofs_y + C, ofs_x * 2 - C, ofs_y / 2 - C, olc::GREEN);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C),(float) (ofs_y + 5 * C)), "CLICK HERE\n\nTO CHANGE\n\nSOURCE", olc::BLACK, olc::vf2d(1.0f, 1.0f));
		FillRect(GWidth + 3 * ofs_x / 2 + 1, 2 * ofs_y + 1, ofs_x * 2 - 1, ofs_y / 2 - 1, olc::DARK_RED);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5),(float) (2 * ofs_y + 5)), "SELECT POINT\n\nTO CHANGE TO\n\nDESTINATION", olc::BLACK, olc::vf2d(1.0f, 1.0f));

	}
	else {
		FillRect(GWidth + 3 * ofs_x / 2 + C, ofs_y + C, ofs_x * 2 - C, ofs_y / 2 - C, olc::GREEN);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C),(float) (ofs_y + 5 * C)), "CLICK HERE\n\nTO CHANGE\n\nSOURCE", olc::BLACK, olc::vf2d(1.0f, 1.0f));
		FillRect(GWidth + 3 * ofs_x / 2 + C, 2 * ofs_y + C, ofs_x * 2 - C, ofs_y / 2 - C, olc::RED);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2  + 5 * C),(float)( 2 * ofs_y + 5 * C)), "CLICK HERE\n\nTO CHANGE\n\nDESTINATION", olc::BLACK, olc::vf2d(1.0f, 1.0f));

	}


	DrawRect(GWidth + 3 * ofs_x / 2, 7 * ofs_y / 2, ofs_x * 2, 2 * ofs_y / 5, olc::DARK_CYAN);
	DrawRect(GWidth + 3 * ofs_x / 2, 9 * ofs_y / 2, ofs_x * 2, 2 * ofs_y / 5, olc::DARK_ORANGE);


	if (flag1 == 1) {
		FillRect(GWidth + 3 * ofs_x / 2 + 1, 7 * ofs_y / 2 + 1, ofs_x * 2 - 1, 2 * ofs_y / 5 - 1, olc::DARK_CYAN);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C), (float)(7 * ofs_y / 2 + 5 * C)), "\nSHOW BOTH \n\n PATHS ", olc::BLACK, olc::vf2d(1.0f, 1.0f));
		FillRect(GWidth + 3 * ofs_x / 2 + C, 9 * ofs_y / 2 + C, ofs_x * 2 - C, 2 * ofs_y / 5 - C, olc::ORANGE);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C), (float)(9 * ofs_y / 2 + 5 * C)), "SHOW ONLY PATH\n\nW/O DIAGONAL\n\nTRAVERSAL", olc::BLACK, olc::vf2d(0.8f, 0.8f));
	}
	else if (flag1 == 2) {
		FillRect(GWidth + 3 * ofs_x / 2 + C, 7 * ofs_y / 2 + C, ofs_x * 2 - C, 2 * ofs_y / 5 - C, olc::CYAN); 
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C),(float) (7 * ofs_y / 2 + 5 * C)), "SHOW ONLY PATH \n\nWITH DIAGONAL \n\nTRAVERSAL ", olc::BLACK, olc::vf2d(0.8f, 0.8f));
		FillRect(GWidth + 3 * ofs_x / 2 + 1, 9 * ofs_y / 2 + 1, ofs_x * 2 - 1, 2 * ofs_y / 5 - 1, olc::DARK_ORANGE);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C),(float) (9 * ofs_y / 2 + 5 * C)), "\nSHOW BOTH\n\n PATHS ", olc::BLACK, olc::vf2d(1.0f, 1.0f));

	}
	else {
		FillRect(GWidth + 3 * ofs_x / 2 + C, 7 * ofs_y / 2 + C, ofs_x * 2 - C, 2 * ofs_y / 5 - C, olc::CYAN);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C), (float)(7 * ofs_y / 2 + 5 * C)), "SHOW ONLY PATH \n\nWITH DIAGONAL \n\nTRAVERSAL ", olc::BLACK, olc::vf2d(0.8f, 0.8f));
		FillRect(GWidth + 3 * ofs_x / 2 + C, 9 * ofs_y / 2 + C, ofs_x * 2 - C, 2 * ofs_y / 5 - C, olc::ORANGE);
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2 + 5 * C),(float) (9 * ofs_y / 2 + 5 * C)), "SHOW ONLY PATH\n\nW/O DIAGONAL\n\nTRAVERSAL", olc::BLACK, olc::vf2d(0.8f, 0.8f));

	}



	
	if (GetMouse(0).bPressed) {

		int NodeWidth = (GWidth / GridCols);
		int NodeHeight = (GHeight / GridRows);
		int Correction = 1;

		int x = GetMouseX();
		int y = GetMouseY();

		if ((x < GWidth + 7 * ofs_x / 2 -  1) && (y < 3 * ofs_y / 2) && (x > GWidth + 3 * ofs_x / 2) && (y > ofs_y)) {
			flag = (flag==1)?0:1;
		}
		else if ( (x < GWidth + 7 * ofs_x / 2 - 1) && (y < 5 * ofs_y / 2) && (x > GWidth + 3 * ofs_x / 2 - 1) && (y > 2 * ofs_y)) {
			flag = (flag==2) ? 0 : 2;
		}

		if ((x < GWidth + 7 * ofs_x / 2 - 1) && (y < 39 * ofs_y / 10) && (x > GWidth + 3 * ofs_x / 2) && (y > 7 * ofs_y / 2)) {
			flag1 = (flag1==1) ? 0 : 1;
		}
		else if ((x < GWidth + 7 * ofs_x / 2 - 1) && (y < 49 * ofs_y / 10) && (x > GWidth + 3 * ofs_x / 2 - 1) && (y > 9 * ofs_y / 2)) {
			flag1 = (flag1==2) ? 0 : 2;
		}


		if(flag) {
				
				if (GetMouse(0).bPressed) {
					int x = GetMouseX() - ofs_x;
					int y = GetMouseY() - ofs_y;
					if (0 < x && x < GWidth - 1 && y > 0 && y < GHeight - 1)
					{
						
						int x1 = x / NodeWidth;
						int y1 = y / NodeHeight;
						if (&Grid[x1][y1] == StartNode || &Grid[x1][y1] == End ) {
						}
						else {
							if (flag == 1) {
								Grid[x1][y1].obstacle = false;
								StartNode = &Grid[x1][y1];
							
							}
							else if (flag == 2) {
								Grid[x1][y1].obstacle = false;
								End = &Grid[x1][y1];
								
							}
						}

						FindPath(SolvedPath, PathLength, PathFound, 1);
						FindPath(SolvedPath_wd, PathLength_wd, PathFound_wd, 2);
						flag= 0;
					}
				}
				 
			}



	}


	
	if (PathFound) {
		string str ="SHORTEST PATH FOUND\n\nWITH\n\nDIAGONAL TRAVERSAL\n\nDistance : ";
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2), (float)(3 * ofs_y) ), str + to_string(PathLength), olc::GREEN, olc::vf2d(0.8f, 0.8f));
	}
	else {
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2),(float) (3 * ofs_y) ), "PATH NOT FOUND\n\nWITH\n\nDIAGONAL TRAVERSAL", olc::RED, olc::vf2d(0.8f, 0.8f));
	}
	if (PathFound_wd) {
		string str = "SHORTEST PATH FOUND\n\nWITHOUT\n\nDIAGONAL TRAVERSAL\n\nDistance : ";
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2), (float)(4 * ofs_y)), str + to_string(PathLength_wd), olc::GREEN, olc::vf2d(0.8f, 0.8f));
	}
	else {
		DrawStringDecal(olc::vf2d((float)(GWidth + 3 * ofs_x / 2),(float) (4 * ofs_y) ), "PATH NOT FOUND\n\nWITHOUT\n\nDIAGONAL TRAVERSAL!", olc::RED, olc::vf2d(0.8f, 0.8f));
	
	}


	

	
}

