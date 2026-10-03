#include "skyrim/extraction/bsa_archive.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

// A development-only CLI exercises the production reader without NIF decoding,
// generation, or desktop startup. Timings exclude this process's startup.
int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: navmesh-bsa-probe archives.txt snapshot requests.txt bulk|incremental|changed|index\n";
        return 1;
    }
    try
    {
        std::ifstream archiveList(argv[1]);
        std::ifstream requestList(argv[3]);
        if (!archiveList || !requestList)
        {
            return 1;
        }
        std::vector<std::filesystem::path> archives;
        for (std::string path; std::getline(archiveList, path);)
        {
            archives.emplace_back(path);
        }
        std::set<std::string> requested;
        for (std::string name; std::getline(requestList, name);)
        {
            requested.insert(name);
        }
        const auto start = std::chrono::steady_clock::now();
        navmesh::skyrim::BsaModelExtractor extractor(std::move(archives), argv[2]);
        bool success = true;
        const std::string mode(argv[4]);
        if (mode == "bulk")
        {
            success = extractor.Extract(requested);
        }
        else if (mode == "incremental")
        {
            for (const auto &name : requested)
            {
                success = extractor.Extract({name}) && success;
            }
        }
        else if (mode == "changed" || mode == "index")
        {
            std::set<std::filesystem::path> changed;
            if (mode == "changed")
            {
                for (const auto &path : requested)
                {
                    changed.emplace(path);
                }
            }
            std::set<std::string> models;
            success = extractor.ChangedModels(changed, models);
            if (success)
            {
                std::ofstream stream(std::filesystem::path(argv[2]) / "changed-models.txt");
                for (const auto &name : models)
                {
                    stream << name << '\n';
                }
            }
        }
        else
        {
            return 1;
        }
        const auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "{\"seconds\":" << seconds << ",\"success\":" << (success ? "true" : "false") << "}\n";
        return success ? 0 : 2;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
