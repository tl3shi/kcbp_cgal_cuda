#pragma once

#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include "CP_PointVector.h"
#include "TextHelper.hpp"
#include <algorithm>

using namespace std;

enum ObjLineType
{
    Unkown, VertexNormal, Vertex, Facet, EndOfFile
};

enum ObjIndexVertexType 
{
    None = 0,
    VertexOnly = 1,
    TextureCoordinates = 2,
    Normal = 4,
    VertexAndTextureCoordinates = VertexOnly | TextureCoordinates,
    VertexAndNormal = VertexOnly | Normal,
    TextureCoordinatesAndNormal = VertexOnly | TextureCoordinates | Normal
};

class ObjReadHelper
{
    public:
       
      ObjLineType static NextLine(char* &buffer)
      {
            //trim
            if(! TextHelper::ParseBlankNewline(buffer))
                return ObjLineType::EndOfFile;
            
            while (true)
            {
                if(*buffer == 'v')
                {
                    buffer++;
                    if(*buffer == 'n') //normal
                    {
                        ;
                    }
                    if(*buffer == ' ')//just v, is vertex
                    {
                        if (!TextHelper::ParseBlank(buffer))
                            return ObjLineType::EndOfFile;
                        return ObjLineType::Vertex;
                    }
                }
                else if(*buffer == 'f' )//facet
                {
                    buffer++;
                    if (!TextHelper::ParseBlank(buffer))
                        return ObjLineType::EndOfFile;
                    return ObjLineType::Facet;
                }
                else
                {
                    if (!TextHelper::GotoNextLine(buffer))
                        return ObjLineType::EndOfFile;
                    if (!TextHelper::ParseBlankNewline(buffer))
                        return ObjLineType::EndOfFile;
                }
            }
      }

      ObjIndexVertexType static ParseIndices(char *&buffer, int values[3])
      {
          char *p = buffer;
          if (!TextHelper::ParseInteger(p, values[0]))
              return ObjIndexVertexType::None;
          TextHelper::ParseBlank(p);
          if (*p != '/')
          {
              buffer = p;
              return ObjIndexVertexType::VertexOnly;
          }
          TextHelper::ParseBlank(++p);
          if (*p == '/')
          {
              TextHelper::ParseBlank(++p);
              if (!TextHelper::ParseInteger(p, values[2]))
                  return ObjIndexVertexType::None;
              buffer = p;
              return ObjIndexVertexType::VertexAndNormal;
          }
          else
          {
              if (!TextHelper::ParseInteger(p, values[1]))
                  return ObjIndexVertexType::None;
              TextHelper::ParseBlank(p);
              if (*p == '/')
              {
                  TextHelper::ParseBlank(++p);
                  if (!TextHelper::ParseInteger(p, values[2]))
                      return ObjIndexVertexType::None;
                  buffer = p;
                  return ObjIndexVertexType::TextureCoordinatesAndNormal;
              }
              else
              {
                  buffer = p;
                  return ObjIndexVertexType::VertexAndTextureCoordinates;
              }
          }
      }
};

class FileManager
{

public: 
	FileManager(){}

	~FileManager(){}
	
    static void outputpoints(const vector<CP_Vector3D> &points, const string &filename)
    {
        ofstream outf(filename.c_str());
        streambuf *default_buf=cout.rdbuf(); 
        cout.rdbuf( outf.rdbuf() ); 
        
        for (unsigned int i = 0; i < points.size(); i++)
            cout << points[i].x << " " << points[i].y << " "  << points[i].z   << endl;

        cout.rdbuf(default_buf);
    }

    static void adjustPointsbak(vector<CP_Vector3D> &points, const double &min_, const double &max_)
    {
        //the opengl be ..
        double scale = 20.0 / (max_ - min_);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            points[i] = points[i] * scale;
        }
    }

    static double getDrawScale(const double &min_, const double max_)
    {
        return 20.0 / (max_ - min_);
    }
    static vector<CP_Vector3D> readCsvNormal(const string &file_name)
    {
        vector<CP_Vector3D> result;
        ifstream fin(file_name.c_str());  
        assert(fin);
        string line;  
        while( getline(fin,line) )
        {    
            if(line.length() > 0)
            {
                vector<string> s = split(line, ",");
                result.push_back(CP_Vector3D(atof(s[0].c_str()), atof(s[1].c_str()), atof(s[2].c_str())));
             }
        }
        return result;
    }

    static vector<CP_Vector3D> readPoints(const string &file_name)
    {
        vector<CP_Vector3D> result;
        ifstream fin(file_name.c_str());  
        assert(fin);
        string line;  
        while( getline(fin,line) )
        {    
            if(line.length() > 0)
            {
                vector<string> s = split(line, " ");
                result.push_back(CP_Vector3D(atof(s[0].c_str()), atof(s[1].c_str()), atof(s[2].c_str())));
            }
        }
        return result;
    }

    bool static readObjPoints(vector<CP_Vector3D> &vecs, string file_name, double & draw_scale,  bool pointsAreOnlyFirstPart = true)
    {
        //only want to load the points, ignore the normal and faces
        bool no_normals = true;
        bool no_faces= true;
        //bool pointsAreOnlyFirstPart = true;

        ifstream fin(file_name.c_str());  
        if(!fin)
        {
            cout << "make sure the filename exist!";
            return false;
        }

        vector<CP_Vector3D> normals;
        vector<int *> faces;

        double xmin = DBL_MAX, xmax = -DBL_MAX;
        string line;  
        while( getline(fin,line) )
        {    
            if (line[0] == 'v')
            {
                double vec[3];

                std::string temp;
                std::stringstream ss(line);
                ss >> temp;
                ss >> vec[0];
                ss >> vec[1];
                ss >> vec[2];

                if (line[1] == ' ') // position
                {
                    xmin = min(vec[0], xmin);
                    xmax = max(vec[0], xmax);
                    vecs.push_back(CP_Vector3D(vec[0],vec[1],vec[2]));
                }
                else if (line[1] == 'n')// normal
                {
                    if(pointsAreOnlyFirstPart)
                    {
                        fin.close();
                        draw_scale = getDrawScale(xmin, xmax);
                        return true;
                    }
                    if(no_normals) continue;
                    normals.push_back(CP_Vector3D(vec[0], vec[1],vec[2]));
                }
            }
            else  if (line[0] == 'f')
            {
                if(pointsAreOnlyFirstPart)
                {
                    fin.close();
                    draw_scale = getDrawScale(xmin, xmax);
                    return true;
                }
                if(no_faces) continue;

                int pidx[3], nidx[3];
                std::string temp;
                std::stringstream ss(line);
                ss >> temp;
                for (int i = 0; i < 3; i++)
                {
                    ss >> pidx[i];
                    ss.seekg(2, std::ios::cur);
                    ss >> nidx[i];
                    pidx[i]--;
                    nidx[i]--;
                }
                int * index = new int[3];
                index[0] = pidx[0];index[1] = pidx[1];index[2] = pidx[2];
                faces.push_back(index);
            }
        }
        
        fin.close();
        draw_scale = getDrawScale(xmin, xmax);
        return true;
    }

    template <typename T>
    bool static readObjPointsFast(vector<T> &vecs, string file_name, double & draw_scale)
    {
        ifstream is(file_name.c_str(), std::ifstream::binary);
        if(is)
        {
            is.seekg (0, is.end);
            int length = is.tellg();
            is.seekg (0, is.beg);
            char* buffer = new char[length];
            is.read(buffer, length);

            char* p = buffer;
            const char* buffer_start = p;
            int vertex_count = 0;
            int normal_count = 0;
            bool parse_end = false;
            while(!parse_end)
            {
                ObjLineType line_type = ObjReadHelper::NextLine(p);
                switch (line_type)
                {
                case  ObjLineType::Vertex:
                    ++vertex_count;
                    break;
                case ObjLineType::VertexNormal:
                    ++normal_count;
                    break;
                case ObjLineType::EndOfFile: // the last one (performance enhancement)
                    parse_end = true;
                    break; 
                default:
                    break;
                }
                TextHelper::GotoNextLine(p);
            }
            //cout << "\nveftex count: " << vertex_count << endl;
            vecs.resize(vertex_count);
            parse_end = false;
            int index = 0;
            double xmin = DBL_MAX, xmax = -DBL_MAX;
            p = buffer;//from the start to parse
            while (!parse_end)
            {
                 ObjLineType line_type = ObjReadHelper::NextLine(p);
                 if(line_type == ObjLineType::Vertex )
                 {
                     T point;
                     TextHelper::Parse(p, point.x);
                     TextHelper::ParseBlank(p);
                     TextHelper::Parse(p, point.y);
                     TextHelper::ParseBlank(p);
                     TextHelper::Parse(p, point.z);
                     TextHelper::ParseBlank(p);
                     vecs[index++] = point;

                     xmin = min(point.x, xmin);
                     xmax = max(point.x, xmax);
                 }
                 else if(line_type == ObjLineType::EndOfFile)
                 {
                    parse_end = true;
                 }
            }
            draw_scale = getDrawScale(xmin, xmax);
            delete[] buffer;
            return true;
        }
        return false;
    }

    template <typename T>
    bool static readObjPointsAndFacetsFast(vector<T> &vecs, vector<int> &faces, string file_name, T &box_min, T &box_max)
    {
        ifstream is(file_name.c_str(), std::ifstream::binary);
        box_min = T(DBL_MAX, DBL_MAX, DBL_MAX);
        box_max = T(-DBL_MAX, -DBL_MAX, -DBL_MAX);
        if(is)
        {
            is.seekg (0, is.end);
            int length = is.tellg();
            is.seekg (0, is.beg);
            char* buffer = new char[length];
            is.read(buffer, length);

            char* p = buffer;
            const char* buffer_start = p;
            int vertex_count = 0;
            int normal_count = 0;
            int face_count = 0;
            bool parse_end = false;
            while(!parse_end)
            {
                ObjLineType line_type = ObjReadHelper::NextLine(p);
                switch (line_type)
                {
                case  ObjLineType::Vertex:
                    ++vertex_count;
                    break;
                case ObjLineType::VertexNormal:
                    ++normal_count;
                    break;
                case ObjLineType::Facet:
                    ++face_count;
                    break;
                case ObjLineType::EndOfFile: // the last one (performance enhancement)
                    parse_end = true;
                    break; 
                default:
                    break;
                }
                TextHelper::GotoNextLine(p);
            }
            //cout << "\nveftex count: " << vertex_count << endl;
            vecs.resize(vertex_count);
            faces.resize(face_count * 3);
            parse_end = false;
            int index = 0;
            int face_v_index = 0;
            p = buffer;//from the start to parse
            while (!parse_end)
            {
                ObjLineType line_type = ObjReadHelper::NextLine(p);
                if(line_type == ObjLineType::Vertex )
                {
                    T point;
                    TextHelper::Parse(p, point.x);
                    TextHelper::ParseBlank(p);
                    TextHelper::Parse(p, point.y);
                    TextHelper::ParseBlank(p);
                    TextHelper::Parse(p, point.z);
                    TextHelper::ParseBlank(p);
                    vecs[index++] = point;
                    box_min.x = min(point.x, box_min.x);
                    box_min.y = min(point.y, box_min.y);
                    box_min.z = min(point.z, box_min.z);
                    box_max.x = max(point.x, box_max.x);
                    box_max.y = max(point.y, box_max.y);
                    box_max.z = max(point.z, box_max.z);
                }else if(line_type == ObjLineType::Facet)
                {
                    while (true)
                    {
                        int indices[3] = { 0, 0, 0 };
                        ObjReadHelper::ParseIndices(p, indices);//only want the face index
                        if(face_v_index == faces.size()) //wrong, for some obj, like apple3.obj
                            faces.push_back(indices[0]-1);
                        else
                            faces[face_v_index++] = indices[0]-1;
                        TextHelper::ParseBlank(p);
                        if (*p == '\\')
                        {
                            TextHelper::GotoNextLine(p);
                            TextHelper::ParseBlank(p);
                        }
                        if ((*p == '\r') || (*p == '\n') || (*p == 0))
                            break;
                    }
                   
                }
                else if(line_type == ObjLineType::EndOfFile)
                {
                    parse_end = true;
                }
            }
            delete[] buffer;
            return true;
        }
        return false;
    }

    template <typename T>
    bool static readObjPointsAndFacetsFast(vector<T> &vecs, vector<int> &faces, string file_name, double &draw_scale)
    {
        ifstream is(file_name.c_str(), std::ifstream::binary);
        if(is)
        {
            is.seekg (0, is.end);
            int length = is.tellg();
            is.seekg (0, is.beg);
            char* buffer = new char[length];
            is.read(buffer, length);

            char* p = buffer;
            const char* buffer_start = p;
            int vertex_count = 0;
            int normal_count = 0;
            int face_count = 0;
            bool parse_end = false;
            while(!parse_end)
            {
                ObjLineType line_type = ObjReadHelper::NextLine(p);
                switch (line_type)
                {
                case  ObjLineType::Vertex:
                    ++vertex_count;
                    break;
                case ObjLineType::VertexNormal:
                    ++normal_count;
                    break;
                case ObjLineType::Facet:
                    ++face_count;
                    break;
                case ObjLineType::EndOfFile: // the last one (performance enhancement)
                    parse_end = true;
                    break; 
                default:
                    break;
                }
                TextHelper::GotoNextLine(p);
            }
            //cout << "\nveftex count: " << vertex_count << endl;
            vecs.resize(vertex_count);
            faces.resize(face_count * 3);
            parse_end = false;
            int index = 0;
            int face_v_index = 0;
            p = buffer;//from the start to parse
            double xmin = DBL_MAX, xmax = -DBL_MAX;
            while (!parse_end)
            {
                ObjLineType line_type = ObjReadHelper::NextLine(p);
                if(line_type == ObjLineType::Vertex )
                {
                    T point;
                    TextHelper::Parse(p, point.x);
                    TextHelper::ParseBlank(p);
                    TextHelper::Parse(p, point.y);
                    TextHelper::ParseBlank(p);
                    TextHelper::Parse(p, point.z);
                    TextHelper::ParseBlank(p);
                    vecs[index++] = point;
                    xmin = min(point.x, xmin);
                    xmax = max(point.x, xmax);
                }else if(line_type == ObjLineType::Facet)
                {
                    /*
                    int v_index = -1;
                    for (int i = 0; i < 3; i++)
                    {
                        TextHelper::Parse(p, v_index);
                        //v_index//normal_index //cannot use this. 
                        TextHelper::ParseBlank(p);
                        faces[face_v_index++] = v_index-1; //obj start from 1
                    }*/
                    while (true)
                    {
                        int indices[3] = { 0, 0, 0 };
                        ObjReadHelper::ParseIndices(p, indices);//only want the face index
                        if(face_v_index == faces.size()) //wrong, for some obj, like apple3.obj
                            faces.push_back(indices[0]-1);
                        else
                            faces[face_v_index++] = indices[0]-1;
                        TextHelper::ParseBlank(p);
                        if (*p == '\\')
                        {
                            TextHelper::GotoNextLine(p);
                            TextHelper::ParseBlank(p);
                        }
                        if ((*p == '\r') || (*p == '\n') || (*p == 0))
                            break;
                    }
                   
                }
                else if(line_type == ObjLineType::EndOfFile)
                {
                    parse_end = true;
                }
            }
            draw_scale = getDrawScale(xmin, xmax);
            delete[] buffer;
            return true;
        }
        return false;
    }



    static vector<string> split(const string& s, const string& delim, const bool keep_empty = true) 
    {
        vector<string> result;
        if (delim.empty()) {
            result.push_back(s);
            return result;
        }
        string::const_iterator substart = s.begin(), subend;
        while (true) {
            subend = search(substart, s.end(), delim.begin(), delim.end());
            string temp(substart, subend);
            if (keep_empty || !temp.empty()) {
                result.push_back(temp);
            }
            if (subend == s.end()) {
                break;
            }
            substart = subend + delim.size();
        }
        return result;
    }
};

