

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <bits/stdc++.h>
#include <time.h> 

using namespace std;
using namespace cv;

class Node_
{
public:
    Point p;
    double dist;

    Node_(int x, int y, double dis)
    {
        this->p = Point(x,y);
        this->dist = dis;
    }
};

// bool Comparator(Node_ a, Node_ b)
// {
//     return a.dist > b.dist;
// }


class Compare {
public:
	bool operator()(Node_ a,Node_ b) {
		return a.dist > b.dist;
	}
};

bool isValid(Mat img,int x,int y) {
	if(x < 0 | y < 0 | x >= img.cols | y >= img.rows) return false;
	if(img.at<uchar>(y,x) == 255) return false;
	return true;
}

double h(Point curr, Point goal)
{
    return sqrt(pow(curr.x - goal.x,2) + pow(curr.y - goal.y,2));
}


void Plan(Mat img,Point source, Point goal)
{
    // Priority Queue for Dijkstra

    bool reached = false;

    Node_ src(source.x,source.y,0);

    priority_queue<Node_, vector<Node_>, Compare> Q;
    // visited, parent, f,g
    vector<vector<int>>  visited(img.cols, vector<int>(img.rows,0));
    vector<vector<Node_>>  parent(img.cols, vector<Node_>(img.rows,src));
    vector<vector<double>>  f(img.cols, vector<double>(img.rows,FLT_MAX));
    vector<vector<double>>  g(img.cols, vector<double>(img.rows,FLT_MAX));
    cout << "nodes pushed";
    Q.push(src);
    g[source.x][source.y] = 0;
    // parent[source.x][source.y] = src;
    parent[source.x][source.y] = Node_(source.x,source.y,0);
    double wt = 1;
    double h_Node_ , f_Node_;
    Mat final_image = img.clone();
    Mat obstacles = img.clone();

    while(!Q.empty())
    {
        Node_ current = Q.top();
        Q.pop();
        visited[current.p.x][current.p.y] = 1;

        for(int s=-1;s<=1;s++){
            for(int u=-1;u<=1;u++){
                if(s==0 & u ==0)
                {
                    continue;
                }
                if(isValid(obstacles,current.p.x+s,current.p.y+u))
                {
                    if(visited[current.p.x + s][current.p.y + u] == 0)
                    {
                        wt = 1;
                        if((fabs(s) + fabs(u)) != 1) wt = 1.414;
                        
                        h_Node_ = h(Point(current.p.x+s,current.p.y+u),goal);
                        f_Node_ = h_Node_ + wt +g[current.p.x][current.p.y];

                        if(f_Node_ < f[current.p.x+s][current.p.y+u]){ 
                            final_image.at<uchar>(current.p.y,current.p.x) = 255;
                            f[current.p.x+s][current.p.y+u] = f_Node_;
                            g[current.p.x + s][current.p.y + u] = g[current.p.x][current.p.y] + wt;
                            parent[current.p.x+s][current.p.y + u] = Node_(current.p.x, current.p.y ,0);

                            Q.push(Node_(current.p.x+s,current.p.y + u,f_Node_));

                        }

                    }
                }
            }
        }
    }

    cout << "End of the loop";
    
    cout << "---------Showing Path--------\n";

    Node_ current = Node_(goal.x,goal.y,0);
    Node_ curr_next = Node_(goal.x,goal.y,0);
    int x_curr = current.p.x, y_curr = current.p.y;

    while(1)
    {
        if(x_curr == source.x & y_curr == source.y)
        {
            break;
        }
        current = Node_(curr_next.p.x, curr_next.p.y,0);
        img.at<char>(current.p.y,current.p.x) = 255;

        curr_next = Node_(parent[current.p.x][current.p.y].p.x,parent[current.p.x][current.p.y].p.y,0);
		x_curr = curr_next.p.x;
		y_curr = curr_next.p.y;
    }
    cout << "COST : " << g[goal.x][goal.y] << endl;
	namedWindow("DJI",WINDOW_AUTOSIZE);
	imshow("DJI",img);
	// imshow("EXPANDED",show_expanding);
	waitKey(0);
	imwrite("Output.png",img);

}



int main(int argc, char *argv[]) {
	clock_t t;
	t = clock();

	Mat img = imread("chinti.png",0);
	Point source(300,450);
	Point goal(300,100);

    if(isValid(img,goal.x,goal.y))
    {
        Plan(img,source,goal);
        cout << "done";
    }
    else{
        cout <<"Invalid Goal" << endl;
    }


	t = clock() - t;
	cout << "Time : " << t << endl;
}


// g++ -o a_star a_star.cpp `pkg-config --cflags --libs opencv4`
// code to run

