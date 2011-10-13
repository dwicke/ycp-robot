// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
//
// This file is part of HOG-Man.
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with HOG-Man.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <string>
#include <cassert>
#include <sstream>

#include "graph/posegraph3d.h"
#include "pose_graph_vis3d.h"
#include "qgl_graph_viewer.h"
#include "main_widget.h"

#include <qapplication.h>
using namespace std;
using namespace AISNavigation;

struct HGraph
{
  PoseGraph3D graph;
  std::vector<PoseGraph3DVis::HEdgeVector> hirarchy;
  void clear()
  {
    graph.clear();
    hirarchy.clear();
  }
};

static HGraph graphs[2];
static bool s_drawNeeded = false;
static pthread_mutex_t graphMutex = PTHREAD_MUTEX_INITIALIZER;
static bool overrideCovariances = false;
static bool keepRootAtZero = true;

void updateDisplayedGraph(PoseGraph3DVis* poseGraph, int graphIdx, int nextIdx)
{
  pthread_mutex_lock( &graphMutex );
  s_drawNeeded = true;
  graphs[graphIdx].clear(); // clear the old graph
  PoseGraph3D* graph = &graphs[nextIdx].graph;
  if (keepRootAtZero) {
    PoseGraph3D::Vertex* rootVertex = static_cast<PoseGraph3D::Vertex*>(graph->vertices().begin()->second);
    Transformation3 rootToZero = rootVertex->transformation.inverse();
    for (PoseGraph3D::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
      PoseGraph3D::Vertex* v = static_cast<PoseGraph3D::Vertex*>(it->second);
      v->transformation = rootToZero * v->transformation;
    }
  }
  poseGraph->setGraph(graph);
  poseGraph->setHirarchy(&graphs[nextIdx].hirarchy);
  pthread_mutex_unlock( &graphMutex );
}

/**
 * this thread will read the stuff that arrives via stdin and create a graph out of it
 */
void* readStdinThread(void* arg)
{
  bool switchedGraph = false;
  PoseGraph3DVis* poseGraph = static_cast<PoseGraph3DVis*>(arg);
  string token, line;
  stringstream auxStream;
  int graphIdx = 0;
  double timestamp = 0.0;

  // read stdin data
  char c = 0;
  while (cin.get(c)) { // if cin is not valid, we are done
    int nextIdx = (graphIdx + 1) & 1;
    PoseGraph3D& nextGraph = graphs[nextIdx].graph;
    std::vector<PoseGraph3DVis::HEdgeVector>& nextHirarchy = graphs[nextIdx].hirarchy;

    if (c == 'V') {
      switchedGraph = false;
      // read vertex
      int id;
      Transformation3 t;
      static Matrix6 identity = Matrix6::eye(1.);
      cin.read((char*)&id, sizeof(int));
      cin.read((char*)&t.translation()[0], sizeof(double));
      cin.read((char*)&t.translation()[1], sizeof(double));
      cin.read((char*)&t.translation()[2], sizeof(double));
      cin.read((char*)&t.rotation().w(), sizeof(double));
      cin.read((char*)&t.rotation().x(), sizeof(double));
      cin.read((char*)&t.rotation().y(), sizeof(double));
      cin.read((char*)&t.rotation().z(), sizeof(double));
      PoseGraph3D::Vertex* v = nextGraph.addVertex(id, t, identity);
      if (! v) {
        cerr << "vertex " << id << " is already in the graph, reassigning "<<  endl;
        v = nextGraph.vertex(id);
        assert(v);
      } 
      v->transformation = t;
      v->localTransformation = t;
    }
    else if (c == 'E') {
      switchedGraph = false;
      // read edge
      int id1, id2;
      Transformation3 t;
      Matrix6 m = Matrix6::eye(1.);
      cin.read((char*)&id1, sizeof(int));
      cin.read((char*)&id2, sizeof(int));
      cin.read((char*)&t.translation()[0], sizeof(double));
      cin.read((char*)&t.translation()[1], sizeof(double));
      cin.read((char*)&t.translation()[2], sizeof(double));
      cin.read((char*)&t.rotation().w(), sizeof(double));
      cin.read((char*)&t.rotation().x(), sizeof(double));
      cin.read((char*)&t.rotation().y(), sizeof(double));
      cin.read((char*)&t.rotation().z(), sizeof(double));
      if (overrideCovariances) {
        double dummy; // just read over the information matrix
        for (int i=0; i<6; i++)
          for (int j=i; j<6; j++)
            cin.read((char*)&dummy, sizeof(double));
      } else {
        for (int i=0; i<6; i++)
          for (int j=i; j<6; j++) {
            cin.read((char*)&m[i][j], sizeof(double));
            if (i != j)
              m[j][i] = m[i][j];
          }
      }

      PoseGraph3D::Vertex* v1 = nextGraph.vertex(id1);
      PoseGraph3D::Vertex* v2 = nextGraph.vertex(id2);
      if (! v1 ) {
        cerr << "vertex " << id1 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
        continue;
      }
      if (! v2 ) {
        cerr << "vertex " << id2 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
        continue;
      }
      PoseGraph3D::Edge* e = nextGraph.addEdge(v1, v2, t, m);
      if (! e){
        cerr << "error in adding edge " << id1 << "," << id2 << endl;
      } 
    }
    else if (c == 'H') { // read hirarchy
      PoseGraph3DVis::HEdge e;
      int l;
      cin.read((char*) &l, sizeof(int));
      cin.read((char*) &e.id1, sizeof(int));
      cin.read((char*) &e.id2, sizeof(int));
      if (l + 1 > (int)nextHirarchy.size())
        nextHirarchy.resize(l+1);
      nextHirarchy[l].push_back(e);
    }
    else if (c == 'T') {
      cin.read((char*) &timestamp, sizeof(double));
    }
    else if (c == 'F') { // finished reading... switch the graph display
      switchedGraph = true;
      updateDisplayedGraph(poseGraph, graphIdx, nextIdx);
      graphIdx = nextIdx;
    }
  }

  // done reading from stdin, check wether there was an END read
  if (!switchedGraph) {
    int nextIdx = (graphIdx + 1) & 1;
    updateDisplayedGraph(poseGraph, graphIdx, nextIdx);
    graphIdx = nextIdx;
  }

  return 0;
}

void printUsage(const char* progName)
{
  cout << "Usage: " << progName << " [options]" << endl << endl;
  cout << "Options:" << endl;
  cout << "-------------------------------------------" << endl;
  cout << "-l <filename>  load a graph from the hard disk" << endl;
  cout << "-b             file from disk is binary" << endl;
  cout << "-o             overwrite covariances" << endl;
  cout << "-h             this help" << endl;
}

int main(int argc, char** argv)
{
  string localFilename = "";
  bool binary = false;

  for (char c; (c = getopt(argc, argv, "bol:h")) != -1; ) {
    switch (c) {
      case 'b':
        binary = true;
        break;
      case 'o':
        overrideCovariances = true;
        break;
      case 'l':
        localFilename = optarg;
        break;
      case 'h':
        printUsage(argv[0]);
        return 0;
      default:
        printUsage(argv[0]);
        return 1;
    }
  }
  if (optind != argc) {
    printUsage(argv[0]);
    return 1;
  }

  // starting up the QApplication
  QApplication qapp(argc, argv);
  MainWidget mw;
  QGLGraphViewer& viewer = *mw.viewer;
  viewer.graph.setUseDrawList(true);
  viewer.graph.setGraph(&graphs[0].graph);
  viewer.graph.setHirarchy(&graphs[0].hirarchy);
  mw.show();

  // start the thread that reads stdin
  pthread_t thread1;
  int thread_status = pthread_create( &thread1, NULL, readStdinThread, static_cast<void*>(&viewer.graph));
  if (thread_status != 0) {
    cerr << "unable to create stdin thread" << endl;
  }

  // just load a graph and display this graph
  if (localFilename.size()) {
    cout << "Loading file " << localFilename << " ... " << flush;
    ifstream ifs(localFilename.c_str());
    if (!ifs) {
      cerr << "Unable to open " << localFilename << endl;
      return 1;
    }
    if (!binary)
      graphs[0].graph.load(ifs);
    else
      graphs[0].graph.loadBinary(ifs);
    cout << "done." << endl;
  }

  while (mw.isVisible()) {
    pthread_mutex_lock( &graphMutex );
    if (s_drawNeeded) {
      s_drawNeeded = false;
      viewer.updateGL();
    }
    qapp.processEvents();
    pthread_mutex_unlock( &graphMutex );
    usleep(10000);
  }

  return 0;
}
