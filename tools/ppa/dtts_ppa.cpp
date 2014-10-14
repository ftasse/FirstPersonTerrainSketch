#include "dtts_ppa.h"
#include "dtts_mst.h"

using namespace Dtts;

bool node_in(const node_t &a, const node_list &list)
{
    return std::find(list.begin(), list.end(), a) != list.end();
}

float ndistance(const node_t &p, const node_t &q)
{
    return sqrt((float)((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y)));
}


Tree::Tree() {
    nedges=0;
    msource = NULL;
}

node_list Tree::getNodes()
{
    node_list nodes;
    for (map_t::const_iterator ite=mtree.begin(); ite!=mtree.end(); ite++)
        nodes.push_back(ite->first);
    return nodes;
}

node_list& Tree::neighbours(const node_t& pnode)
{
    return (mtree.find(pnode))->second;
}

node_list Tree::neighbours_(const node_t& pnode)
{
    return (mtree.find(pnode))->second;
}

node_list Tree::getcontrolpts(const node_t& pnode)
{
    return (cpts.find(pnode))->second;
}

node_list Tree::bfs_full(const node_t& rnode)
{
    node_list visited,result;
    queue<node_t> Q;
    node_t pnode = rnode;

    while (result.size()!=mtree.size())
    {

        int maxdg = 0;
        for (map_t::const_iterator ite=mtree.begin(); ite!=mtree.end(); ite++)
        {
            int deg = getcontrolpts(ite->first).size();
            if ( deg>maxdg && (!node_in(ite->first,result)) )
            {
                maxdg = deg;
                pnode = ite->first;
            }
        }

        Q.push(pnode);
        visited.clear();

        while (!Q.empty())
        {
            node_t q = Q.front();

            if ( getcontrolpts(q).size()>0 && !node_in(q,result) )
            {
                result.push_back(q);
            }
            Q.pop();

            const node_list& out =  neighbours(q);
            for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
            {
                if ( !node_in(*it,visited) )
                {
                    Q.push(*it);
                    visited.push_back(*it);
                }

            }
        }
    }

    return result;
}

node_list Tree::bfs(const node_t& pnode)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(pnode);

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( !node_in(q,result) )
        {
            result.push_back(q);
        }
        Q.pop();

        const node_list& out =  neighbours(q);
        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( !node_in(*it,visited) )
            {
                Q.push(*it);
                visited.push_back(*it);
            }

        }
    }

    return result;
}

node_list Tree::bfs_iso(const node_t& pnode)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(pnode);

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( !node_in(q,result) )
        {
            result.push_back(q);
        }
        Q.pop();

        const node_list& out =  isofeatures.find(q)->second;
        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( !node_in(*it,visited) )
            {
                Q.push(*it);
                visited.push_back(*it);
            }

        }
    }

    return result;
}

bool redo = false;

node_t Tree::bfs_short(const node_t& parent, const node_t& child,node_list& visited, float rad)
{
    node_list result;
    queue<node_t> Q;
    Q.push(child);
    visited.push_back(child);
    //node_t prev=parent;

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( !node_in(q,result) )
        {

            result.push_back(q);
            if ( ndistance(parent,q)>=rad )  return q;

        }
        Q.pop();

        const node_list& out =  neighbours(q);

        //if (out.size()>2)   return q;

        if (out.size()>0){
            /*node_t next(-1,-1);
            {
                float maxheight=0;
                //cout<<"\t\t\tout: ";
                for (node_list::const_iterator it=out.begin(); it!=out.end(); it++){
                    //cout<<(*it).x-parent.x+50<<"/"<<(*it).y-parent.y+50<<" ";
                }
                //cout<<endl;
                for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
                {

                    if ( *it!= parent && msource((*it).x,(*it).y)>maxheight && !node_in(*it,visited) ) // && msource((*it).x,(*it).y)>maxheight
                    {
                        //Q.push(*it);
                        //visited.push_back(*it);
                        next = *it;
                        maxheight = msource((*it).x,(*it).y);
                        //Q.push(next);
                        //visited.push_back(next);
                    }

                }

            }*/

            /*if  (next.x>0){
                //cout<<"\t\t\tnext: "<<next.x-parent.x+50<<"/"<<next.y-parent.y+50<<endl;
                Q.push(next);
                visited.push_back(next);
            }*/
            for (node_list::const_iterator it=out.begin(); it!=out.end(); it++){
                if ( *it!= parent && !node_in(*it,visited)){
                    Q.push(*it);
                    visited.push_back(*it);
                }
            }
        }

    }

    return node_t(-1,-1);    //dummy node
}

node_list Tree::bfs_short2(const node_t& parent, const node_t& child, node_list& visited, float rad)
{
    node_list result;
    queue<node_t> Q;
    Q.push(child);
    visited.push_back(child);
    //node_t prev=parent;
    //cout<<"\tParent and child: "<<parent.x<<"/"<<parent.y<<" and "<<child.x<<"/"<<child.y<<endl;

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( !node_in(q,result) )
        {

            result.push_back(q);
            //cout<<"\t\t"<<q.x-parent.x+50<<"/"<<q.y-parent.y+50<<endl;
            if ( ndistance(parent,q)>=rad )  return result;

        }
        Q.pop();



        const node_list& out =  neighbours(q);  if (out.size()>2)   return result;
        if (out.size()>0){
            node_t next(-1,-1);
            {
                float maxheight=0;
                //cout<<"\t\t\tout: ";
                for (node_list::const_iterator it=out.begin(); it!=out.end(); it++){
                    //cout<<(*it).x-parent.x+50<<"/"<<(*it).y-parent.y+50<<" ";
                }
                //cout<<endl;
                for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
                {

                    if ( *it!= parent && msource->getAltitude((*it).x,(*it).y)>maxheight && !node_in(*it,visited) ) // && msource((*it).x,(*it).y)>maxheight
                    {
                        //Q.push(*it);
                        //visited.push_back(*it);
                        next = *it;
                        maxheight = msource->getAltitude((*it).x,(*it).y);
                        //Q.push(next);
                        //visited.push_back(next);
                    }

                }

            }

            if  (next.x>0){
                //cout<<"\t\t\tnext: "<<next.x-parent.x+50<<"/"<<next.y-parent.y+50<<endl;
                Q.push(next);
                visited.push_back(next);
            }
        }
    }
    return result;    //dummy node
}

node_list Tree::branchlength(const node_t& parent, const node_t& child, unsigned int minimum)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(child);

    visited.push_back(parent);
    visited.push_back(child);

    while (!Q.empty())
    {
        node_t q = Q.front();
        const node_list& out =  neighbours(q);

        if ( !node_in(q,result) )
        {
            if (out.size()>2){
               return result; //getNodes();
            }

            result.push_back(q);

            if (out.size()==1 )
                return result;

            if (result.size()>minimum)  return getNodes();

        }
        Q.pop();

        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( !node_in(*it,visited) )
            {
                Q.push(*it);
                visited.push_back(*it);
            }

        }
    }

    return result;
}

node_list Tree::branchlength2(const node_t& parent, const node_t& child, unsigned int minimum)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(child);

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( !node_in(q,result) )
        {

            result.push_back(q);
            if (result.size()>minimum)  return getNodes();

        }
        Q.pop();

        const node_list& out =  neighbours(q);
        if (out.size()==1 || out.size()>2)
            return result;

        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( *it!= parent && !node_in(*it,visited) )
            {
                Q.push(*it);
                visited.push_back(*it);
            }

        }
    }

    return result;
}

void Tree::getRoots(int bsize){

    //cout<<"Roots:\n";
    //for (int k=0; k<roots.size();k++)
    //   //cout<<"\t"<<roots[k].x<<"/"<<roots[k].y<<"\n";

   for (unsigned int k=0; k<roots.size();k++){
       //cout<<"\t"<<roots[k].x<<"/"<<roots[k].y<<"\n";
       isofeatures[roots[k]];
       processNodes.push_back(roots[k]);
       branchpath(roots[k],bsize);

   }
  //cout<<"Process Nodes: "<<processNodes.size()<<endl;
  //cout<<"All Nodes: "<<getNodes().size()<<endl;

}

node_t Tree::branchpath(node_t pnode, node_t child, int bsize)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(child);

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( q!=pnode && !node_in(q,result)  )
        {

            result.push_back(q);
            //if (result.size()>minimum)  return getNodes();

        }
        Q.pop();

        const node_list& out =  neighbours(q);
        if ( q!=pnode && (out.size()==1 || out.size()>2)){

            if (isofeatures.find(q)==isofeatures.end()){
                isofeatures[q];
                (isofeatures.find(pnode)->second).push_back(q);
                (isofeatures.find(q)->second).push_back(pnode);
                //pathfeatures.push_back(result);
                node_t prev = pnode;
                for (node_list::const_iterator itx=result.begin(); itx!=result.end(); itx++)
                {
                    if ( (!node_in(*itx,processNodes)) && getcontrolpts(*itx).size()==2  && ndistance(*itx,prev)>bsize/2 )  //
                    {
                        processNodes.push_back(*itx);
                        prev = *itx;
                    }

                }
                if ((!node_in(q,processNodes))&& getcontrolpts(q).size()==out.size())  processNodes.push_back(q); // && getcontrolpts(q).size()>0
                return q;
            }
            else{
                 (isofeatures.find(pnode)->second).push_back(q);
                (isofeatures.find(q)->second).push_back(pnode);
                return node_t(-1,-1);
            }
        }
        else

        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( *it!= pnode &&  *it!= child && !node_in(*it,visited) )
            {
                Q.push(*it);
                visited.push_back(*it);
            }

        }
    }

    return node_t(-1,-1);
}


void Tree::branchpath(node_t pnode,int bsize)
{
    node_list visited,result;
    queue<node_t> Q;
    Q.push(pnode);

    while (!Q.empty())
    {
        node_t q = Q.front();

        if ( q!=pnode && !node_in(q,result)  )
        {

            result.push_back(q);
            //if (result.size()>minimum)  return getNodes();

        }
        Q.pop();

        node_list out = neighbours(q);

        for (node_list::const_iterator it=out.begin(); it!=out.end(); it++)
        {
            if ( *it!= pnode && !node_in(*it,visited) )
            {
                node_t newq = branchpath(q,*it,bsize);
                if (newq.x>0 ){
                    Q.push(newq);
                    visited.push_back(newq);
                }
            }

        }
    }

    return;
}

void Tree::loadFile(const char* fname)
{
    std::ifstream s (fname);
    if (!s.is_open())
    {
       //cout<< "Could not open features file "<<fname<<endl;
        return;
    }

    while (!s.eof())
    {
        int x1,y1,x2,y2;
        s >> x1 >> y1 >> x2 >> y2 ;
        node_t p1(x1,y1);
        node_t p2(x2,y2);
        add_edge(p1,p2);
    }
    s.close();

        node_list nodes = getNodes();   //int bef=nodes.size();
    map<node_t, bool> marker;

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {
            marker[*it]=false;
    }

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {

        node_t  pnode = *it;
        if ( marker[*it] )   continue;

        node_list group = bfs(pnode);

        int maxi = -1;
        node_t troot;
        for (node_list::const_iterator ct= group.begin(); ct!=group.end(); ct++){
            marker[*ct] = true;
            int nsize = neighbours(*ct).size();
            if (nsize>maxi && nsize!=2){
                maxi = nsize;
                troot = *ct;
            }
        }

        if (maxi>0){
              roots.push_back(troot);
              continue;
        }
    }

}


void Tree::saveFile(const char* fname)
{
    std::ofstream s (fname);
    if (!s.is_open())
    {
       //cout<< "Could not open features file "<<fname<<endl;
        return;
    }

    node_list nodes = getNodes();

    for (node_list::const_iterator it1 = nodes.begin(); it1!=nodes.end(); it1++)
    {
        for (node_list::const_iterator it2 = it1; it2!=nodes.end(); it2++)
        {
            if ( is_edge(*it1,*it2) )
            {
                s<< it1->x <<" "<< it1->y <<" "<< it2->x <<" "<< it2->y <<endl;
            }
        }
    }
    s.close();
}

void Tree::saveFile_branches(const char* fname,unsigned int deg)
{
    std::ofstream s (fname);
    if (!s.is_open())
    {
       //cout<< "Could not open features file "<<fname<<endl;
        return;
    }

    node_list nodes = getNodes();

    for (node_list::const_iterator it1 = nodes.begin(); it1!=nodes.end(); it1++)
    {
        node_list leafs = getcontrolpts(*it1);
        if (leafs.size()==deg)
            for (node_list::const_iterator it2 = leafs.begin(); it2!=leafs.end(); it2++)
            {
                s<< it1->x <<" "<< it1->y <<" "<< it2->x <<" "<< it2->y <<endl;
            }
    }
    s.close();
}


bool Tree::is_node(node_t pnode)
{
    if (mtree.find(pnode)==mtree.end()) return false;
    return true;
}

bool Tree::is_edge(const node_t& pnode1, const node_t& pnode2)
{
    if ( !is_node(pnode1) ) return false;
    if ( !is_node(pnode2) ) return false;

    const node_list& out =  neighbours(pnode1);
    if ( find(out.begin(),out.end(),pnode2) == out.end() ) return false;
    return true;
}

void Tree::add_edge(const node_t& pnode1, const node_t& pnode2)
{
    if ( !is_node(pnode1) ) mtree[pnode1];
    if ( !is_node(pnode2) ) mtree[pnode2];

    if (!is_edge(pnode1,pnode2))
    {
        neighbours(pnode1).push_back(pnode2);
        neighbours(pnode2).push_back(pnode1);
        nedges++;
    }
}

void Tree::rem_edge(const node_t& pnode1, const node_t& pnode2)
{
    if (is_edge(pnode1,pnode2))
    {
        neighbours(pnode1).remove(pnode2);
        neighbours(pnode2).remove(pnode1);
        nedges--;

    }
}

void Tree::rem_node(const node_t& pnode)
{
    if ( is_node(pnode) )
    {

        node_list out = neighbours(pnode);
        for (node_list::const_iterator it = out.begin(); it!=out.end(); it++){
            rem_edge(pnode,*it);
        }

        mtree.erase(pnode);

    }
}


node_list Tree::control_pts(const node_t& pnode, const float radius)
{
    node_list result;
    const node_list& out = neighbours(pnode);
    //vector<node_list> vlist;

    node_list visited;
    //node_list visited2;


    for ( node_list::const_iterator it = out.begin(); it != out.end(); it++ )
    {
        node_t leaf = bfs_short(pnode,*it,visited,radius);
        //node_list nlist = bfs_short2(pnode,*it,visited2,radius);

        //if ((leaf.x>=0 && ndistance(leaf,pnode)>=radius) )  vlist.push_back(nlist);
        if ((leaf.x>=0) && (ndistance(leaf,pnode)>=radius || out.size()!=2) )  result.push_back(leaf);
    }

    /*if (pnode.x==305 && pnode.y==425){
       //cout<<"Exemple\n";
        if (!node_in(pnode,getNodes())){
           //cout<<"What?\n";
        }
        for ( node_list::const_iterator itx = result.begin(); itx != result.end(); itx++ )
           //cout<<(*itx).x<<"/"<<(*itx).y<<endl;

    }*/
    //followers[pnode] = vlist;


    return result;
}




//node_list getSuccessors

void Tree::compute_control_pts(const float radius)
{

    node_list nodes = getNodes();   //int bef=nodes.size();
    map<node_t, bool> marker;

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {
            marker[*it]=false;
    }

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {

        node_t  pnode = *it;
        if ( marker[*it] )   continue;

        node_list group = bfs(pnode);

        int maxi = -1;
        node_t troot;
        for (node_list::const_iterator ct= group.begin(); ct!=group.end(); ct++){
            marker[*ct] = true;
            int nsize = neighbours(*ct).size();
            if (nsize>maxi && nsize!=2){
                maxi = nsize;
                troot = *ct;
            }
        }

        if (maxi>0){
              roots.push_back(troot);
        }
  }
  root = roots[0];

    //cout<<"Compute control points\n";
    nodes = getNodes();
    unsigned int maxdeg=0;

    //cout<<root.x<<" "<<root.y<<endl;

    for ( node_list::const_iterator it = nodes.begin(); it != nodes.end(); it++ )
    {
        node_list leafs = control_pts(*it,radius);
        cpts[*it] = leafs;
        if (leafs.size()>maxdeg)
        {
            maxdeg = leafs.size();
            root = *it;
        }

        node_t pnode = *it;

    }

   ////cout<<root.x<<" "<<root.y<<endl;

    //for ( node_list::const_iterator it = nodes.begin(); it != nodes.end(); it++ ){
    //    node_list successors = getSuccessors(*it);
     //   if (successors.size()>0){
     //       isofeatures[*it] = successors;
     //   }
    //}


}


class Edge
{
public:
    node_t node1,node2;
    float weight;
    bool status;
    bool visited;
    Edge()
    {
        visited=false;
        status=false;
        weight=0;
    }
    Edge(node_t n1, node_t n2, float mweight):node1(n1),node2(n2),weight(mweight)
    {
        status =true;
        visited = false;
    }
};
bool** profileRecognition(Terrain &img, int plength, int srate);
bool** segmentCreation(Terrain &img, bool** candidates,int srate, float ethres);
vector<Edge> polygonBreaking (Terrain& img, bool** segments, int srat, float ethres);

//void formCycle(vector<Segment>& segments, Segment seg);


void Tree::runPPA(bool is_ridge, int plength, int radius,int groupmin, int branchmin, int samplerate, float ethres)
{
    clock_t Start_t, End_t;

    ridge_type = is_ridge;

    //if (!is_ridge){
     //       for (int i=0; i<src.width(); i++)
     //           for (int j=0; j<src.height(); j++){
     //               src(i,j) = -src(i,j);
    //                //cout<<i<<"/"<<j<<" "<<src(i,j)<<endl;
     //           }
    //}

    Terrain img(*msource,samplerate); //img.save("/tmp/sample.tiff");
    //if (!is_ridge) img.reverse();  //TODO Uncomment this to support valleys

   //cout<< "Profile Recognition ... \n";
    Start_t = clock();
    bool** candidates = profileRecognition(img,plength,samplerate);
    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)/CLOCKS_PER_SEC<<" s\n";


   //cout<< "Segment Creation ... \n";
    Start_t = clock();
    bool** segments = segmentCreation(img,candidates,samplerate,ethres);
    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //renderCandidates(msource,candidates,segments,1.);
    int w = img.width();
    for (int i=0; i<w; i++) delete [] candidates[i];
    delete [] candidates;

   //cout<< "Polygon Breaking  ...\n";
    Start_t = clock();
    vector<Edge> edges = polygonBreaking(img,segments,samplerate,ethres);
    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";
    //cout<< "End Polygon Breaking ( "<<edges.size()<<" edges)\n";

    //Fill the features tree
    for (unsigned short i=0; i<edges.size(); i++)
    {
        if (edges[i].status)
        {
            add_edge(edges[i].node1,edges[i].node2);
            //src.drawline(edges[i].node1,edges[i].node2,0);
        }
    }
    if (getNodes().size()==0){
       //cout<<"Nothing\n";
        return;

    }
    //renderFeatures(1.).save("/tmp/ppa_normal_tree.tiff");


    /*cout<< "Cleaning up ...\n";
    //cout<< "Cleaning up branches  ...\n";
    Start_t = clock();
    //cleanBranches(radius/samplerate);
    //for(int i=0; i<radius/samplerate; i++)
    //    cleanBranches(i+1);
    //cleanBranches2(branchmin);
    cleanBranches((branchmin/samplerate)/2);
    cleanBranches((branchmin/samplerate));
    //    (plength/2);
    //cout<< "Cleaning up clusters  ...\n";
    //cleanBranches2((radius/samplerate));
    cleanGroups(groupmin);
    //renderFeatures(1.).save("/tmp/ppa_normal_reduc.tiff");

    //renderFeatures(1.).save("/tmp/PPA_reduction.tiff");
    Smoothing();

    //renderFeatures(1.).save("/tmp/ppa_normal_smooth.tiff");

    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";*/

        //cleanBranches((portion/srate)/2);
   //cout<< "Branch reduction  ...\n";
    Start_t = clock();
    //cleanBranches(2); //(portion/srate)



    cleanBranches(samplerate);   //renderFeatures(0.).save("/tmp/ppa_shaun_branch.tiff");
    //cleanBranches(5);

    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //cleanBranches(portion/srate);
    //for(int i=0; i<5; i++)
        //cleanBranches2(portion);
    //    (plength/2);
    //cout<< "Cleaning up clusters  ...\n";
    //cleanBranches2((radius/samplerate));

    //End_t = clock();
    //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";
    //renderFeatures(210.).save("/tmp/ppa_shaun_reduc.tiff");

   //cout<< "Smoothing  ...\n";
    Start_t = clock();
    Smoothing();
    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";



   //cout<< "Group reduction  ...\n";
    Start_t = clock();
    //cleanGroups(nedges/10);
    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //renderFeatures(0.).save("/tmp/ppa_shaun_smooth.tiff");

}


//1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8-N

int nx[8] = {0,1,1,1,0,-1,-1,-1};
int ny[8] = {1,1,0,-1,-1,-1,0,1};

bool** profileRecognition(Terrain &img, int plength, int srate)
{
    int w = img.width(), h = img.height();
    //float range = (img.getMax()-img.getMin())*0.5;

    bool** cands = new bool* [w];
    for (int i=0; i<w; i++)
    {
        cands[i] = new bool [h];
        for (int j=0; j<h; j++){
            cands[i][j]=false;
            //if (img.getAltitude(i,j)>range)                cands[i][j]=true;
        }
    }
    //return cands;

    bool lt[8];
    int p,q;
    float thres = 0.5;//0.001*(img.getMax()-img.getMin());

   //cout<<"Threshold: "<<thres<<endl;

    for (int i=0; i<w; i++)
        for (int j=0; j<h; j++)
        {
            //cout<<i<<" "<<j<<endl;

            for (int k=0; k<8; k++)
            {
                lt[k]=false;

                for (int m=0; m<plength/2; m++)
                {
                    p = i + m * nx[k];
                    q = j + m * ny[k];
                    if ( img.validCoordinates(p,q) )
                        if  (img.getAltitude(i,j)-img.getAltitude(p,q)>thres)
                            lt[k]=true;
                }
            }

            for (int m=0; m<4; m++)
                if (lt[m] && lt[m+4])
                {
                    cands[i][j]=true;
                }
        }
    return cands;
}



// ****************************   Segment Creation    *********************************************** /



int neb(int k)
{
    while (k<0)
        k+=4;
    while (k>=4)
        k-=4;
    return k;
}

void RidgeSeg(Terrain& img, bool** segments, int i, int j, int k)
{
    int w = img.width();

    int p = i+nx[k];
    int q = j+ny[k];
    float h = img.getAltitude(i,j)+img.getAltitude(p,q);
    if (k==1 || k==3)
    {
        int kd = neb(k+1);
        int x1 = i+nx[kd];
        int y1 = j+ny[kd];
        float h1 = img.getAltitude(x1,y1)*2;
        kd = neb(k-1);
        int x2 = i+nx[kd];
        int y2 = j+ny[kd];
        float h2 = img.getAltitude(x2,y2)*2;
        if (h>h1 && h>h2)
        {
            segments[x1+y1*w][k]=false;
            segments[x2+y2*w][k]=false;
            segments[x1+y1*w][neb(k+4)]=false;
            segments[x2+y2*w][neb(k+4)]=false;
            //cout<<"hi!\n";
        }
    }
    else
    {
        int kd = neb(k+2);
        int x1 = i+nx[kd];
        int y1 = j+ny[kd];
        float h1 = img.getAltitude(x1,y1)+img.getAltitude(p+nx[kd],q+ny[kd]);
        kd = neb(k-2);
        int x2 = i+nx[kd];
        int y2 = j+ny[kd];
        float h2 = img.getAltitude(x2,y2)+img.getAltitude(p+nx[kd],q+ny[kd]);
        if (h>h1 && h>h2)
        {
            segments[x1+y1*w][k]=false;
            segments[x2+y2*w][k]=false;
            //cout<<"ha!\n";
        }
    }
}

bool** segmentCreation(Terrain &img, bool** candidates, int srate, float ethres)
{
    int w = img.width(), h = img.height();

    bool** segments = new bool* [w*h];
    for (int m=0; m<w*h; m++)
    {
        segments[m] = new bool [4];
        for (int k=0; k<4; k++)
            segments[m][k]=false;
    }
    for (int j=1; j<h-1; j++)
        for (int i=1; i<w-1; i++)
        {
            for (int k=0; k<4; k++)
            {
                segments[i+j*w][k]=false;
                if (candidates[i][j] && candidates[ i+nx[k] ][ j+ny[k] ] )
                {
                    //printf("new segment\n");
                    segments[i+j*w][k]=true;
                }
                if (k==3 && segments[i+(j-1)*w][1] )
                {
                    if (img.getAltitude(i,j)+img.getAltitude(i+1,j-1) > img.getAltitude(i,j-1)+img.getAltitude(i+1,j))
                        segments[i+(j-1)*w][1]=false;
                    else
                        segments[i+j*w][3]=false;
                }

            }

        }
    // Remove inadequate parallel segments
    for (int j=2; j<h-2; j++)
        for (int i=2; i<w-2; i++)
        {
            for (int k=0; k<4; k++)
                RidgeSeg(img,segments,i,j,k);
        }

    return segments;
}

// ****************************   End of Segment Creation    *********************************************** /


// ****************************  Polygon Breaking ********************************************** /


vector<Edge> br_edges;
map< node_t, vector<int> > polygons;

bool existsPath(mark_t& marker, node_t v1,node_t v2)
{
    if(v1 == v2) return true;
    marker[v1] = true; //marker[v1.first+v1.second*w] = true;

    vector<int> nlist  = polygons[v1];
    //cout<<v1.x<<"-"<<v1.y<<": "<<polygons[v1].size()<<endl; cin.get();
    for (unsigned int i=0; i<nlist.size(); i++)
    {
        int k = nlist[i];
        if (!br_edges[k].status)   continue;
        node_t nnext;
        if (v1==br_edges[k].node1)  nnext = br_edges[k].node2;
        else nnext = br_edges[k].node1;

        if (!marker[nnext])
            if ( existsPath(marker,nnext,v2) )   return true;
    }
    return false;
}

bool formsCycle(mark_t& marker, int x)
{
    for (mark_t::const_iterator it = marker.begin(); it!=marker.end(); it++)            marker[it->first] = false;
    //printf("Edge %d is valid 2\n",x);
    br_edges[x].status=false;

    bool res = existsPath(marker, br_edges[x].node1,br_edges[x].node2);

    br_edges[x].status=true;
    return res;
}

bool lessEdge(Edge ed1, Edge ed2)
{
    return (ed1.weight<ed2.weight);
}

vector<Edge> polygonBreaking (Terrain& img, bool** segments, int rate, float ethres)
{
    br_edges.clear();
    polygons.clear();

    vector<node_t> knodes;

    //cout<<"sample rate: "<<rate<<endl;

    for (int i=0; i<img.width(); i++)
        for (int j=0; j<img.height(); j++)
            for (int k=0; k<4; k++)
                if ( segments[i+j*img.width()][k] )
                {
                    node_t n1( i*rate, j*rate);
                    node_t n2( (i+nx[k])*rate, (j+ny[k])*rate );
                    float mweight = img.getAltitude(i,j) + img.getAltitude(i+nx[k],j+ny[k]);

                    //if (abs(mweight)>abs(ethres*img.getMax()))
                    {
                        br_edges.push_back(Edge(n1,n2,mweight));
                        //cout<< n1.x <<" "<< n1.y <<" "<< n2.x <<" "<< n2.y <<endl; cin.get();
                    }
                }
    //s.close();  //exit(0);

    for (int wh=0; wh<img.width()*img.height(); wh++) delete [] segments[wh];
    delete [] segments;

    //cout<<"Edges size: "<<br_edges.size()<<endl;


    mark_t marker;
    for (unsigned int k=0; k<br_edges.size(); k++)
    {
        if (marker.find(br_edges[k].node1)==marker.end())
        {
            marker[br_edges[k].node1]=false;
            polygons[br_edges[k].node1];
            knodes.push_back(br_edges[k].node1);
        }
        if (marker.find(br_edges[k].node2)==marker.end())
        {
            marker[br_edges[k].node2]=false;
            polygons[br_edges[k].node2];
            knodes.push_back(br_edges[k].node2);
        }
    }

    sort(br_edges.begin(),br_edges.end(),lessEdge);

    //node_t nodex(480,30);

    //cout<<polygons[nodex].size()<<endl;
    //cout<<"Nodes size: "<<knodes.size()<<endl;

    //for (int n=0; n<knodes.size(); n++){
    //    for (int k=0; k<br_edges.size(); k++){
    //       node_t node1 = br_edges[k].node1;
    //       node_t node2 = br_edges[k].node2;
     //      if (knodes[n]==node1 || knodes[n]==node2)  polygons[knodes[n]].push_back(k);
    //    }
    //}

    for (unsigned int k=0; k<br_edges.size(); k++)
    {
        node_t node1 = br_edges[k].node1;
        node_t node2 = br_edges[k].node2;

        //if (nodex==node1)  {cout<<k; polygons[node1].push_back(k); cin.get();}
        //if (nodex==node2)  {cout<<k; polygons[node2].push_back(k); cin.get();}

        polygons[node1].push_back(k);
        polygons[node2].push_back(k);
        //cout<<node1.x<<"/"<<node1.y<<"-"<<node2.x<<"/"<<node2.y<<": "<<k<<" "<<polygons[node1].size()<<" "<<polygons[node2].size()<<endl;
    }

    //cout<<nodex.x<<"/"<<nodex.y<<endl;
    //vector<int> nlist = polygons[nodex];
    //for (int n=0; n<nlist.size(); n++){
    //     int k = nlist[n];
    //     node_t node1 = br_edges[k].node1;
    //     node_t node2 = br_edges[k].node2;
    //    //cout <<k<<": "<<node1.x<<"/"<<node1.y<<"-"<<node2.x<<"/"<<node2.y<<endl;
    //}
    //cout<<polygons[nodex].size()<<endl;

    //cout<<"Nodes size: "<<knodes.size()<<endl;

    //*std::ofstream s ("/home/flora/grapheg.txt");
    //for (int k=0; k<br_edges.size(); k++){
    //    vector<int> edlist = polygons[k];
    //    for (int l=0; l<edlist.size(); l++)
     //      s<< k <<" "<< edlist[l] <<endl;
    //}
    //s.close();

   //cout<<"Edges: "<<br_edges.size()<<endl;

    for (unsigned int k=0; k<br_edges.size(); k++)
    {
        ////cout<<k<<" nb of neighbours: "<<polygons[k].size()<<endl;
        //if (!br_edges[k].status)    continue;

        bool cycle = formsCycle(marker,k);
        //cout<<k<< " result: "<<cycle<<endl;
        if (cycle)
        {
            //cout<<k<<" Edge invalid 1\n";
            br_edges[k].status=false;
            //cin.get();
        }
    }

    //for (unsigned short i=0; i<edges.size(); i++){
    //    //if (i>2000)	break;

    //	if ( edges[i].status && formsCycle(edges,marker,i) ){
    //		//printf("Edge %d is invalidated\n\n",i);
    //		edges[i].status=false;
    //	}
    //}

    return br_edges;

}



// *****************************  Polygon Breaking ********************************************** //


 //cout<<id<<" --> "<< m+(n*mw)<<endl;
                        /*node_t d(i-m,j-n);
                        float cz = (img(i,j)+img(m,n))/2.;
                        float az = img.getAltitude(i+d.x,j+d.y);
                        float bz = img.getAltitude(m-d.x,n-d.y);
                        float kd = (bz-cz)-(cz-az);

                        node_t dp(d.y,-d.x);
                        float az1 = img.getAltitude(i+dp.x,j+dp.y);
                        float bz1 = img.getAltitude(m+dp.x,n+dp.y);
                        float az2 = img.getAltitude(i-dp.x,j-dp.y);
                        float bz2 = img.getAltitude(m-dp.x,n-dp.y);
                        float cz1 = (az1+bz1)/2.;
                        float cz2 = (az2+bz2)/2.;
                        float kdp = (cz-cz2)-(cz1-cz);

                        float ew = -kdp;
                        //cout<<ew<<endl;
                        if (kd!=0) ew/=abs(kd);*/


void Tree::shaunPPA(Terrain& src, bool ridges, int srate, int plength, int portion){

    //clock_t tstart, tstop;
    //tstart = clock();
    clock_t Start_t, End_t;

    //plength = srate*5;

    msource = new Terrain(src, 1);
    Terrain img(*msource,srate);

    int mw = img.width();
    int mh = img.height();
    int i,j;

    if (!ridges){
        for (i=0; i<mw; i++)
            for (j=0; j<mh; j++){
                img.setAltitude(i,j,-img.getAltitude(i,j));
                //cout<<i<<"/"<<j<<endl;
            }
    }

    bool* cands = new bool [mw*mh];

    //cout<<"Profile Recognition, number of nodes: "<<mw*mh<<" ...\n";
     Start_t = clock();


        bool lt[8];
        int p,q;

        float thres = 0.01*(src.max_altitude()-src.min_altitude());
        //float rg = (src.getMax()-src.getMin())/2;

        //if (ridges) thres*=-1.0f;


        for (int j=0; j<mh; j++)  for (int i=0; i<mw; i++)
        {
            for (int k=0; k<8; k++)
                {
                    lt[k]=false;

                    for (int m=0; m<plength/2; m++)
                    {
                        p = i + m * nx[k];
                        q = j + m * ny[k];
                        if ( img.validCoordinates(p,q) )
                            if  (img.getAltitude(i,j)>img.getAltitude(p,q) && abs((img.getAltitude(i,j)-img.getAltitude(p,q))) >thres )//
                                lt[k]=true;
                            //else if  ((!ridges) && img.getAltitude(i/srate,j/srate)-img.getAltitude(p,q)>thres)
                            //    lt[k]=true;
                    }
                }
               cands[i+j*mw]  = false;

                    for (int m=0; m<4; m++)
                        if (lt[m] && lt[m+4])
                        {
                            cands[i+j*mw] = true;
                        }
                    //if (ridges&&abs(img(i,j))>(src.getMax()-src.getMin())*0.25) cands[i+j*mw] = true;
                    //else if ((!ridges)&&abs(img(i,j))<(src.getMax()-src.getMin())*0.75) cands[i+j*mw] = true;

        }

    int w=mw, h=mh;
     bool** segments = new bool* [w*h];
    for (int m=0; m<w*h; m++)
    {
        segments[m] = new bool [4];
        for (int k=0; k<4; k++)
            segments[m][k]=false;
    }
    for (int j=1; j<h-1; j++)
        for (int i=1; i<w-1; i++)
        {
            for (int k=0; k<4; k++)
            {
                segments[i+j*w][k]=false;
                if (cands[i+j*w] && cands[ (i+nx[k])+(j+ny[k])*w ] )
                {
                    //printf("new segment\n");
                    segments[i+j*w][k]=true;
                }
                if (k==3 && segments[i+(j-1)*w][1] )
                {
                    if (img.getAltitude(i,j)+img.getAltitude(i+1,j-1) > img.getAltitude(i,j-1)+img.getAltitude(i+1,j))
                        segments[i+(j-1)*w][1]=false;
                    else
                        segments[i+j*w][3]=false;
                }

            }

        }
    // Remove inadequate parallel segments
    for (int j=2; j<h-2; j++)
        for (int i=2; i<w-2; i++)
        {
            for (int k=0; k<4; k++)
                RidgeSeg(img,segments,i,j,k);
        }

    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";


	cout<<"Graph  construction, number of nodes: "<<mw*mh<<" ...\n";
    Start_t = clock();
    vector<Edge_t> E;

	for (int j=1; j<mh-1; j++)
        for (int i=1; i<mw-1; i++)
        {

         for (int k = 0; k < 4; ++k){
                    int m = i+nx[k];
                    int n = j+ny[k];


                    if ( m>=0 && n>=0 && m<mw && n<mh && segments[i+j*mw][k])
                    {
                        Vertex_t v1(i*srate,j*srate, i+(j*mw) + 1  );
                        Vertex_t v2(m*srate,n*srate, m+(n*mw) + 1 );

                            float weight = (img.getAltitude(i,j)+img.getAltitude(m,n))/2.;
                            Edge_t e( v1, v2, weight );
                            E.push_back(e);

                    }
               }

	}

	for (int id=0; id<mw*mh; id++)  delete [] segments[id];
	delete [] segments;

    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //renderFeatures(0.).save("/tmp/ppa_shaun_all.tiff");


	cout<<"Kruskal start, number of edges: "<<E.size()<<" ...\n";
    Start_t = clock();

    int ecount = E.size();
    //Edge_t* _E = &E[0];
	Edge_t* _E = kruskal2(&E[0],mw*mh,E.size(), &ecount);//kruskal(&V[0], &E[0], V.size(), ecount);

   //cout<<"Kruskal over, number of edges: "<<ecount<<endl;
    End_t = clock();
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    for (int i=0; i<ecount; i++){
        add_edge(node_t(_E[i].V1.x,_E[i].V1.y),node_t(_E[i].V2.x,_E[i].V2.y));
    }

    delete [] _E;

    //renderFeatures(150.).save("/tmp/ppa_shaun_tree.tiff");

   //cout<< "Removed non candidates  ...\n";
    Start_t = clock();


   /*{
    node_list remnodes;
	node_list nodes =  getNodes();

	for (node_list::iterator it = nodes.begin(); it!=nodes.end(); it++){
	   // int i = (*it).x;
	   // int j = (*it).y;

        if (!cands[((*it).x/srate)+((*it).y/srate)*mw])  remnodes.push_back(*it);
        //if ( img((*it).x/srate,(*it).y/srate) > rg)remnodes.push_back(*it);

	}

	for (node_list::iterator it = remnodes.begin(); it!=remnodes.end(); it++){
        rem_node(*it);
	}
  }*/
   delete [] cands;




    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";
    //renderFeatures(150.).save("/tmp/ppa_shaun_clean1.tiff");

   //cout<< "Skelonization  ...\n";
    Start_t = clock();
    //cleanBranches2(50);
    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //cleanBranches((portion/srate)/2);
   //cout<< "Branch reduction  ...\n";
    Start_t = clock();
    //cleanBranches(2); //(portion/srate)



    cleanBranches(plength);   //renderFeatures(150.).save("/tmp/ppa_shaun_branch.tiff");
    //cleanBranches(5);

    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //cleanBranches(portion/srate);
    //for(int i=0; i<5; i++)
        //cleanBranches2(portion);
    //    (plength/2);
    //cout<< "Cleaning up clusters  ...\n";
    //cleanBranches2((radius/samplerate));

    //End_t = clock();
    //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";
    //renderFeatures(210.).save("/tmp/ppa_shaun_reduc.tiff");

   //cout<< "Smoothing  ...\n";
    Start_t = clock();
    Smoothing();
    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

   //cout<< "Group reduction  ...\n";
    Start_t = clock();
    //cleanGroups(nedges/10);
    End_t = clock();
   //cout<<"(V,E) ---> ("<<getNodes().size()<<","<<nedges<<")\n";
   //cout<< "End of Operation ... "<<(float(difftime(End_t,Start_t))/CLOCKS_PER_SEC)<<" s\n";

    //renderFeatures(150.).save("/tmp/ppa_shaun_smooth.tiff");
    //cin.get();
    //getComponents();

    //renderFeatures(0.).save("/tmp/ppa_shaun_smooth.tiff");

    //cin.get();


    delete msource;
}

void Tree::extractFromSet(Terrain& src, std::set<node_t> nodes)
{
    for (std::set<node_t>::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        mtree[*it];
    }

    int nx[8] = {0,1,1,1,0,-1,-1,-1};
    int ny[8] = {1,1,0,-1,-1,-1,0,1};
    int h = src.height(), w = src.width();

    //for (int j = 0; j < 2;++j)
    {
        vector<Edge_t> E;
        for (std::set<node_t>::iterator it = nodes.begin(); it != nodes.end(); ++it)
        {
            node_t node  = *it;
            int i = node.x, j = node.y;

            for (int k=0; k<3; k++)
            {
                node_t other(node_t(i+nx[k],j+ny[k]));
                if (is_node(other))
                {
                    Vertex_t v1(node.x, node.y, node.x+(node.y*w) + 1  );
                    Vertex_t v2(other.x, other.y, other.x+(other.y*w) + 1  );
                    float weight = (src.getAltitude(other.x,other.y)+src.getAltitude(node.x,node.y))/2.;

                    Edge_t e( v1, v2, weight );
                    E.push_back(e);
                }
            }
        }

        int ecount = E.size();
        Edge_t* _E = &E[0];
        //Edge_t* _E = kruskal2(&E[0],w*h,E.size(), &ecount);

        for (int i=0; i<ecount; i++){
            add_edge(node_t(_E[i].V1.x,_E[i].V1.y),node_t(_E[i].V2.x,_E[i].V2.y));
        }

        Smoothing();
    }
}

void Tree::Smoothing(){

    node_list lnodes = getNodes();   //int bef = nodes.size();

    for (node_list::const_iterator it = lnodes.begin(); it!=lnodes.end(); it++ )
    {
        node_t pnode = *it;
        int nx=pnode.x;
        int ny=pnode.y;

        node_list out =  neighbours_(pnode);

            for (node_list::const_iterator itx=out.begin(); itx!=out.end(); itx++)
            {
                nx += (*itx).x;
                ny += (*itx).y;
            }
        nx/=(out.size()+1);
        ny/=(out.size()+1);
        rem_node(pnode);
        pnode = node_t(nx,ny);
        for (node_list::const_iterator itx=out.begin(); itx!=out.end(); itx++)
            {
                add_edge(pnode,*itx);
            }

    }

}

void Tree::cleanBranches(unsigned int branchmin)
{
    //cleanBranches2(75);
    {
    //renderFeatures(250.).save("/tmp/ppa_shaun_branch2.tiff");
     bool stop = false;
    int ko=0;
    //while (!stop)
    for (int m=branchmin; m>0; m-=branchmin/2)
    {

            stop = true;
            node_list nodes = getNodes(),erasen;

            for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
            {
                node_t pnode = *it;
                ko++;
                //if (node_in(*it,erasen))   continue;

                node_list out =  neighbours_(pnode);

                if ( out.size() == 1 )
                {
                    //cout<<pnode.x<<" "<<pnode.y<<endl;

                    node_list tmp = branchlength(pnode,*out.begin(),branchmin);

                    //cout<<ko-1<<": "<<tmp.size()<<" vs "<<branchmin<<endl;

                        //cin.get();

                        if (tmp.size()< (unsigned int) m)
                        {
                            if (tmp.size()==0){
                                //rem_node(*it);
                                 stop = false;
                                if (!node_in(*it,erasen))
                                erasen.push_back(*it);
                            }
                            else{
                                for (node_list::const_iterator itx=tmp.begin(); itx!=tmp.end(); itx++){
                                    //rem_node(*itx);
                                     stop = false;
                                    if (!node_in(*itx,erasen))
                                    erasen.push_back(*itx);
                                }
                            }

                        }
                }
                else if (out.size()==0){
                    //rem_node(*it);
                    stop = false;
                    if (!node_in(*it,erasen))
                    erasen.push_back(*it);
                }
                else{

                }

            }

            for (node_list::const_iterator itx=erasen.begin(); itx!=erasen.end(); itx++)
                rem_node(*itx);

    }
    //cleanBranches2(75);
    return;
    }
    //,erasen;   //int bef = nodes.size();

    //renderFeatures(210).save("/tmp/0pass.tiff");
    //for (int i=0; i<branchmin; i++){
    //cleanBranches2(45);

    /*map<node_t,int> level;
    node_list nodes = getNodes(),erasen;
    int L=1;

    //node_list nodes = getNodes(),erasen;   //int bef = nodes.size();
    for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
        level[*it] = L;
    //cout<<"Level: "<<L<<" "<<nedges<<endl;

    while (L<=3){
        node_list lnodes = getNodes();
        for (node_list::const_iterator it = lnodes.begin(); it!=lnodes.end(); it++ ){
            if ( neighbours(*it).size()==1 && level.find(*it)->second<=L){
                node_t out =  *(neighbours_(*it).begin());
                level[out]++;
                //cout<<"Remove: "<<(*it).x<<"/"<<(*it).y<<endl;
                rem_node(*it);
            }
        }
        L++;
    }

    node_list lnodes = getNodes();
        for (node_list::const_iterator it = lnodes.begin(); it!=lnodes.end(); it++ ){
        node_t pnode = *it;

        if ( is_node(*it) && neighbours(pnode).size() == 1 )
        {

            node_list out =  neighbours(pnode);

            node_list tmp = branchlength(pnode,*out.begin(),branchmin);

                //cout<<tmp.size()<<" vs "<<branchmin<<endl;
                //cin.get();

                if (tmp.size()<branchmin)
                {
                    for (node_list::const_iterator itx=tmp.begin(); itx!=tmp.end(); itx++)
                        rem_node(*itx);

                }

        }

    }

    return;

    while(true){
        bool  stop = true;

    node_list nodes = getNodes(),erasen;

            for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ ){
                if (is_node(*it)){
                    const node_list& out = neighbours(*it);
                    if (out.size()==1 && neighbours(*out.begin()).size()>2){
                        rem_node(*it);
                        stop = false;
                        }

                }

            }
            if (stop)   break;
    }

    return;*/

    /*map<node_t,int> level;
    int L=1;
    node_list nodes = getNodes(),erasen;   //int bef = nodes.size();
    for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
        level[*it] = L;
    //cout<<"Level: "<<L<<" "<<nedges<<endl;

    while (L>branchmin){
        node_list lnodes = getNodes();
        for (node_list::const_iterator it = lnodes.begin(); it!=lnodes.end(); it++ ){
            if ( neighbours(*it).size()==1 && level.find(*it)->second<=L){
                node_t out =  *(neighbours_(*it).begin());
                level[out]++;
                //cout<<"Remove: "<<(*it).x<<"/"<<(*it).y<<endl;
                rem_node(*it);
            }
        }
        L++;
    }*/

    /*while(true){
        bool  stop = true;
        node_list nodes = getNodes(),erasen;

            for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ ){
                if (is_node(*it)){
                    const node_list& out = neighbours(*it);
                    if (out.size()==1 && neighbours(*out.begin()).size()>2){
                        rem_node(*it);
                        stop = false;
                        }

                }

            }
            if (stop)   break;
    }*/


    /*for (int k=branchmin; k>0; k--)
    {
            node_list nodes = getNodes(),erasen;
    for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
            {
                node_t pnode = *it;
                //ko++;
                if (!is_node(*it))   continue;

                node_list out =  neighbours_(pnode);

                if ( out.size() == 1 )
                {
                    //cout<<pnode.x<<" "<<pnode.y<<endl;

                    node_list tmp = branchlength(pnode,*out.begin(),branchmin);

                    //cout<<ko-1<<": "<<tmp.size()<<" vs "<<branchmin<<endl;

                        //cin.get();

                        if (tmp.size()<branchmin)
                        {
                            if (tmp.size()==0){
                                //rem_node(*it);
                                 if (!node_in(*it,erasen))
                                rem_node(*it);
                            }
                            else{
                                for (node_list::const_iterator itx=tmp.begin(); itx!=tmp.end(); itx++){
                                    //rem_node(*itx);
                                    if (!node_in(*itx,erasen))
                                    rem_node(*it);
                                }
                            }

                        }
                }
                else if (out.size()==0){
                    //rem_node(*it);
                   rem_node(*it);
                }
                else{

                }

            }
    }
    return;


    bool stop = false;
    int ko=0;
    while (!stop){

            stop = true;
            node_list nodes = getNodes(),erasen;


            for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
            {
                node_t pnode = *it;
                ko++;
                //if (node_in(*it,erasen))   continue;

                node_list out =  neighbours_(pnode);

                if ( out.size() == 1 )
                {
                    //cout<<pnode.x<<" "<<pnode.y<<endl;

                    node_list tmp = branchlength(pnode,*out.begin(),branchmin);

                    //cout<<ko-1<<": "<<tmp.size()<<" vs "<<branchmin<<endl;

                        //cin.get();

                        if (tmp.size()<branchmin)
                        {
                            if (tmp.size()==0){
                                //rem_node(*it);
                                 stop = false;
                                if (!node_in(*it,erasen))
                                erasen.push_back(*it);
                            }
                            else{
                                for (node_list::const_iterator itx=tmp.begin(); itx!=tmp.end(); itx++){
                                    //rem_node(*itx);
                                     stop = false;
                                    if (!node_in(*itx,erasen))
                                    erasen.push_back(*itx);
                                }
                            }

                        }
                }
                else if (out.size()==0){
                    //rem_node(*it);
                    stop = false;
                    if (!node_in(*it,erasen))
                    erasen.push_back(*it);
                }
                else{

                }

            }

            for (node_list::const_iterator itx=erasen.begin(); itx!=erasen.end(); itx++)
                rem_node(*itx);

    }*/

    //renderFeatures(210).save("/tmp/1pass.tiff");
    //cin.get();
    /*for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
    {
        node_t pnode = *it;

        if ( neighbours(pnode).size() == 1 )
        {

            node_list out =  neighbours_(pnode);

            node_list tmp = branchlength(pnode,*out.begin(),branchmin);

                //cout<<tmp.size()<<" vs "<<branchmin<<endl;
                //cin.get();

                if (tmp.size()<branchmin)
                {
                    for (node_list::const_iterator itx=tmp.begin(); itx!=tmp.end(); itx++)
                        if (!node_in(*itx,erasen))
                            erasen.push_back(*itx) ;//rem_node(*itx);

                }

        }

    }*/
    //for (node_list::const_iterator itx=erasen.begin(); itx!=erasen.end(); itx++)
    //    rem_node(*itx);

    //cout <<"hello\n";

    //int now = getNodes().size();

    //printf("size of erasen nodes %d remaining %d\n",bef-now,now);
}

void Tree::cleanBranches2(unsigned int branchmin)
{
    map<node_t,int> level;
    float fraction =  branchmin/100.;
    int L=1;
    int e_max = ( (float)nedges*fraction);

    node_list nodes = getNodes(),erasen;   //int bef = nodes.size();
    for (node_list::const_iterator it = nodes.begin(); it!=nodes.end(); it++ )
        level[*it] = L;


    while (nedges>e_max){
        node_list lnodes = getNodes();
        for (node_list::const_iterator it = lnodes.begin(); it!=lnodes.end(); it++ ){
            if ( neighbours(*it).size()==1 && level.find(*it)->second<=L){
                node_t out =  *(neighbours_(*it).begin());
                level[out]++;
                //cout<<"Remove: "<<(*it).x<<"/"<<(*it).y<<endl;
                rem_node(*it);
            }
        }
       //cout<<"Level: "<<L<<" "<<nedges<<endl;
        L++;
    }
    //cout<<"End: "<<nedges<<endl;
}


void Tree::cleanGroups(unsigned int groupmin)
{
    node_list nodes = getNodes();   //int bef=nodes.size();
    map<node_t, bool> marker;

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {
            marker[*it]=false;
    }

    for (node_list::const_iterator it=nodes.begin(); it != nodes.end(); it++)
    {

        node_t  pnode = *it;
        if ( marker[*it] )   continue;

        node_list group = bfs(pnode);

        int maxi = -1;
        node_t troot;
        for (node_list::const_iterator ct= group.begin(); ct!=group.end(); ct++){
            marker[*ct] = true;
            int nsize = neighbours(*ct).size();
            if (nsize>maxi && nsize!=2){
                maxi = nsize;
                troot = *ct;
            }
        }

        if (group.size()>=groupmin && maxi>0){
              roots.push_back(troot);
              continue;
        }
        for (node_list::const_iterator ct= group.begin(); ct!=group.end(); ct++)
            rem_node(*ct);
    }

    //int now = getNodes().size();
    //printf("size of erasen nodes %d remaining %d\n",bef-now,now);
}

void Dtts::computeBranches(Dtts::Tree &ridges, std::vector<node_list> &branches)
{
    node_list nodes = ridges.getNodes();
    std::map<node_t, bool> marked;
    for (node_list::const_iterator it1 = nodes.begin(); it1!=nodes.end(); it1++)
    {
        marked[*it1] = false;
    }

    for (node_list::const_iterator it1 = nodes.begin(); it1!=nodes.end(); it1++)
    {
        node_list leafs = ridges.neighbours(*it1);
        if (leafs.size()==2) continue;
        if (leafs.size()==1 && marked[*it1]) continue;

        for (node_list::const_iterator it2 = leafs.begin(); it2!=leafs.end(); it2++)
        {
            if (marked[*it2]) continue;

            node_list branch;
            branch.push_back(*it1);
            std::queue<node_t> frontier; frontier.push(*it2);

            while (!frontier.empty())
            {
                node_t cur = frontier.front(); frontier.pop();
                branch.push_back(cur);
                node_list next = ridges.neighbours(cur);

                if (next.size() > 2)
                    break;
                else
                    marked[cur] = true;

                for (node_list::const_iterator it3 = next.begin(); it3!=next.end(); it3++)
                {
                    if (!marked[*it3] && *it3!=*it1)
                    {
                        frontier.push(*it3);
                        if (ridges.neighbours(*it3).size() <= 2)
                            marked[*it3] = true;
                    }
                }
            }

            if (branch.size() >= 2) branches.push_back(branch);
        }
    }
}
