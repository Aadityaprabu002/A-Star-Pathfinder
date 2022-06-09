#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include<iostream>
#include<unordered_set>
#include<vector>
#include<math.h>
#include<time.h>
using namespace std;
class Pathfinder : public olc::PixelGameEngine
{
	private:
		struct node{
			int x;
			int y;
			double h;
			double g;
			double f;
			bool obstacle;
			bool visited;
			node* parent;
			vector<node*> neighbours;
			node(int x = 0,int y = 0) {
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
		
		node** Grid;
		node* End;
		node* StartNode;
		int GridRows;
		int GridCols;
		float ObstaclePopulation;
		vector<node*> OpenSet;
		vector<node*> ClosedSet;
		vector<node*> SolvedPath;
		double Heuristics(node* Neighbour, node* End) {
			return abs(Neighbour->x - End->x) + abs(Neighbour->y - End->y);
			//return sqrt(pow(Neighbour->x - End->x, 2) + pow(Neighbour->y - End->y, 2));
		}
		bool OnUserCreate() override
		{
			
			Grid = new node * [GridRows];
			for (int i = 0; i < GridRows; i++) {
				Grid[i] = new node[GridCols];
			}
		
			StartNode = &Grid[0][0];
			End = &Grid[GridRows - 1][GridCols - 1];
			for (int i = 0; i < GridRows; i++) {
				for (int j = 0; j < GridCols; j++) {
					if (i > 0) {
						Grid[i][j].neighbours.push_back(&Grid[i - 1][j]);
					}
					if (i  < GridRows - 1) {
						Grid[i][j].neighbours.push_back(&Grid[i + 1][j]);
					}
					if (j > 0) {
						Grid[i][j].neighbours.push_back(&Grid[i][j-1]);
					}
					if (j  < GridCols - 1) {
						Grid[i][j].neighbours.push_back(&Grid[i][j + 1]);
					}
					if (i  > 0 and j  > 0) {
						Grid[i][j].neighbours.push_back(&Grid[i - 1][j  - 1]);
					}
					if (i < GridRows - 1 and j < GridCols - 1) {
						Grid[i][j].neighbours.push_back(&Grid[i + 1][j + 1]);
					}
					if (i > 0 and j < GridCols - 1) {
						Grid[i][j].neighbours.push_back(&Grid[i - 1][j + 1]);
					}
					if (j > 0 and i < GridRows - 1) {
						Grid[i][j].neighbours.push_back(&Grid[i + 1][j - 1]);
					}

					Grid[i][j].x = i;
					Grid[i][j].y = j;
					
					float RandomPopulation = (float)rand() / RAND_MAX;
					cout << RandomPopulation <<" "<< ObstaclePopulation<< endl;
					if (RandomPopulation < ObstaclePopulation and &Grid[i][j] != StartNode and &Grid[i][j] != End) {
						Grid[i][j].obstacle = true;
					}
				}
			}
			
			
			return true;
		}
		void FindPath() {

			SolvedPath.clear();
			OpenSet.clear();
			ClosedSet.clear();
			OpenSet.push_back(StartNode);


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
					while (Temp->parent) {
						SolvedPath.push_back(Temp);
						Temp = Temp->parent;
					}
					cout << "Found path!\n";
					break;
				}

				for (auto itr = OpenSet.begin(); itr != OpenSet.end(); itr++) {
					if (*itr == Current) {
						OpenSet.erase(itr);
						break;
					}
				}
		
				ClosedSet.push_back(Current);

				for (node* Neighbour : Current->neighbours) {

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
		bool OnUserUpdate(float fElapsedTime) override
		{
			
			
			int NodeWidth = (ScreenWidth() / GridCols );
			int NodeHeight = (ScreenHeight() / GridRows );

			int Correction = 1;
			int mx = GetMouseX() ;
			int my = GetMouseY();
		
			if (GetMouse(0).bPressed) {
			
				int x = GetMouseX();
				int y = GetMouseY();
				if (0 < x && x < ScreenWidth() - 1) {
					int i = x / NodeWidth;
					int j = y / NodeHeight;
					if (i == 0 and j == 0 || i == GridRows-1 and j ==  GridCols-1) {
						;
					}
					else {
						Grid[i][j].obstacle = !Grid[i][j].obstacle;
						FindPath();
					}
						
				}
			}
			for (int i = 0; i < GridRows; i++) {
				for (int j = 0; j < GridCols; j++)
				{
					FillRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::WHITE);

					if ( SolvedPath.end() != find(SolvedPath.begin(),SolvedPath.end(),&Grid[i][j])) {
						FillRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::Pixel(0,255,255));
					}
					if (&Grid[i][j] == StartNode) {
						FillRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::GREEN);
					}
					if (&Grid[i][j] == End) {
						FillRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::RED);
					}
					if (Grid[i][j].obstacle) {
						FillRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::DARK_GREY);
					}
					DrawRect((i * NodeWidth), (j * NodeHeight), NodeWidth - Correction, NodeHeight - Correction, olc::Pixel(0, 0, 0));
				}
			}
			return true;
		}

	

	public:
		Pathfinder(int GridRows = 0 ,int GridCols = 0,float ObstaclePopulation = 0.4)
		{
			this->sAppName = "A* PATH FINDING ALGORITHM!";
			this->GridRows = GridRows;
			this->GridCols = GridCols;
			this->ObstaclePopulation = ObstaclePopulation;
		}
		~Pathfinder() {
			for (int i = 0; i < GridRows; i++) {
					delete [] Grid[i];
			}
			delete[] Grid;
		}
};
int main()
{
	Pathfinder P(20,20,0.2f);
	if (P.Construct(400,400, 10, 10))
		P.Start();
	return 0;
}
