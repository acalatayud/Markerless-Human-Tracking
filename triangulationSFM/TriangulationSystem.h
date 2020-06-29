//
// Created by lmikolas on 2/4/20.
//

#ifndef TRIANGULATION_TRIANGULATIONSYSTEM_H
#define TRIANGULATION_TRIANGULATIONSYSTEM_H


#include <vector>
#include <fstream>
#include "Camera.h"

class TriangulationSystem {
    public:
        std::vector<std::shared_ptr<std::vector<std::shared_ptr<cv::Point3d>>>> triangulatedFrames;
        std::shared_ptr<std::vector<std::shared_ptr<Camera>>> cameras;

        void setCameras(std::string pathInstrinsics, std::string pathExtrinsics);
        void setFrames(const std::map<std::string, std::string>&  framePaths);
        std::shared_ptr<std::vector<std::shared_ptr<Camera>>> getBestCameras(int frame, int marker);
        void triangulatePoints();
        void exportXYZ();
        void exportCSV();

        static std::vector<std::vector<std::string>> getRowsOfCSV(std::string path) {
            std::ifstream ifs(path.c_str());
            if(!ifs) {
                std::cout << "Failed to open the file." << std::endl;
                exit(0);
            }
            // Read the Data from the file
            // as String Vector
            std::vector<std::vector<std::string>> rows;
            std::string t;
            std::string word;
            int rowCounter = 0;
            while(!safeGetLine(ifs, t).eof()) {
                // adding empty row for store
                std::vector<std::string> row;
                rows.push_back(row);
                // used for separating csv words
                std::istringstream s(t);
                // add words as columns
                while (std::getline(s, word, ',')) {
                    rows[rowCounter].push_back(word);
                }
                rowCounter++;
            }
            return rows;
        }

        // method extracted from https://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
        static std::istream& safeGetLine(std::istream& is, std::string& t) {
            t.clear();

            // The characters in the stream are read one-by-one using a std::streambuf.
            // That is faster than reading them one-by-one using the std::istream.
            // Code that uses streambuf this way must be guarded by a sentry object.
            // The sentry object performs various tasks,
            // such as thread synchronization and updating the stream state.

            std::istream::sentry se(is, true);
            std::streambuf* sb = is.rdbuf();

            for(;;) {
                int c = sb->sbumpc();
                switch (c) {
                    case '\n':
                        return is;
                    case '\r':
                        if(sb->sgetc() == '\n')
                            sb->sbumpc();
                        return is;
                    case std::streambuf::traits_type::eof():
                        // Also handle the case when the last line has no line ending
                        if(t.empty())
                            is.setstate(std::ios::eofbit);
                        return is;
                    default:
                        t += (char)c;
                }
            }
        }
};


#endif //TRIANGULATION_TRIANGULATIONSYSTEM_H
