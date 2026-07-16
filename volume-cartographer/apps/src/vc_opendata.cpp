// vc_opendata — list and pull Vesuvius Challenge open-data ("streamable")
// volpkgs from the command line, reusing the exact machinery the VC3D GUI
// uses: the hosted manifest (OpenDataManifest) for listing, and the segment
// cache engine (OpenDataSegmentCache) for downloading tifxyz segments with a
// correctly synthesized meta.json.
//
//   vc_opendata list      [--json] [--manifest=<url|file>]
//   vc_opendata segments  <sampleId> [--json] [--cache=<dir>] [--manifest=...]
//   vc_opendata pull      <sampleId> [segmentId...] [--smallest=N]
//                         [--cache=<dir>] [--manifest=...] [--force]
//
// Pulled segments land under <cache>/open_data/segments/<sample>/<vol>/<id>/
// as ordinary tifxyz directories usable by every vc_* tool.
//
// Exit codes: 0 ok, 1 usage, 2 failure.

#include "OpenDataManifest.hpp"
#include "OpenDataSegmentCache.hpp"

#include "vc/core/types/VolumePkg.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace vc3d;
using namespace vc3d::opendata;

namespace
{

int usage(const char* argv0)
{
    std::cout
        << "usage:\n"
        << "  " << argv0 << " list      [--json] [--manifest=<url|file>]\n"
        << "  " << argv0 << " segments  <sampleId> [--json] [--cache=<dir>]\n"
        << "  " << argv0 << " pull      <sampleId> [segmentId...] [--smallest=N]\n"
        << "                            [--cache=<dir>] [--force]\n"
        << "\n"
        << "Lists the streamable open-data volpkgs (scrolls/fragments) VC3D\n"
        << "knows about, and downloads their tifxyz segments into a local\n"
        << "cache (default ./vc_opendata_cache) for use with any vc_* tool.\n";
    return 1;
}

OpenDataManifest loadManifest(const std::string& manifestArg)
{
    if (!manifestArg.empty() && fs::exists(manifestArg)) {
        std::cout << "manifest: " << manifestArg << " (local file)\n";
        return loadOpenDataManifestFile(manifestArg);
    }
    const std::string url =
        manifestArg.empty() ? std::string(kDefaultManifestUrl) : manifestArg;
    std::cout << "manifest: " << url << "\n";
    return fetchOpenDataManifest(url);
}

const OpenDataSample* findSampleById(
    const OpenDataManifest& manifest, const std::string& id)
{
    for (const auto& sample : manifest.samples) {
        if (sample.id == id)
            return &sample;
    }
    return nullptr;
}

std::uint64_t segmentCells(const OpenDataSegment& seg)
{
    const int w = seg.width.value_or(0);
    const int h = seg.height.value_or(0);
    if (w <= 0 || h <= 0)
        return UINT64_MAX;  // unknown sizes sort last
    return static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
}

std::string trimTo(const std::string& s, size_t n)
{
    if (s.size() <= n)
        return s;
    return s.substr(0, n - 3) + "...";
}

int cmdList(const OpenDataManifest& manifest, bool json)
{
    if (json) {
        std::cout << "[";
        bool first = true;
        for (const auto& s : manifest.samples) {
            if (!first)
                std::cout << ",";
            first = false;
            bool streamable = false;
            for (const auto& v : s.volumes)
                if (preferredVolumeArtifact(v))
                    streamable = true;
            std::cout << "{\"id\":\"" << s.id << "\",\"type\":\"" << s.type
                      << "\",\"volumes\":" << s.volumes.size()
                      << ",\"tifxyz_segments\":" << s.tifxyzSegmentCount()
                      << ",\"streamable\":" << (streamable ? "true" : "false")
                      << "}";
        }
        std::cout << "]\n";
        return 0;
    }

    std::cout << "\n" << std::left << std::setw(28) << "sample" << std::setw(10)
              << "type" << std::setw(8) << "vols" << std::setw(10) << "tifxyz"
              << "description\n";
    for (const auto& s : manifest.samples) {
        bool streamable = false;
        for (const auto& v : s.volumes)
            if (preferredVolumeArtifact(v))
                streamable = true;
        std::cout << std::left << std::setw(28) << s.id << std::setw(10)
                  << s.type << std::setw(8)
                  << (std::to_string(s.volumes.size()) +
                      (streamable ? "*" : ""))
                  << std::setw(10) << s.tifxyzSegmentCount()
                  << trimTo(s.description, 60) << "\n";
    }
    std::cout << "\n(" << manifest.samples.size()
              << " samples; * = streamable zarr volume available)\n";
    return 0;
}

int cmdSegments(
    const OpenDataManifest& manifest, const std::string& sampleId, bool json,
    const fs::path& cacheRoot)
{
    const OpenDataSample* sample = findSampleById(manifest, sampleId);
    if (!sample) {
        std::cerr << "error: sample '" << sampleId << "' not in the manifest "
                  << "(see `vc_opendata list`)\n";
        return 2;
    }

    if (json)
        std::cout << "[";
    bool first = true;
    int shown = 0;
    for (const auto& seg : sample->segments) {
        if (!seg.hasTifxyz())
            continue;
        const auto* art = preferredTifxyzArtifact(seg);
        const auto state = cacheStateForSegment(cacheRoot, *sample, seg);
        if (json) {
            if (!first)
                std::cout << ",";
            first = false;
            std::cout << "{\"id\":\"" << seg.id << "\",\"width\":"
                      << seg.width.value_or(0)
                      << ",\"height\":" << seg.height.value_or(0)
                      << ",\"cache\":\""
                      << cacheStateName(state) << "\",\"url\":\""
                      << (art ? art->resolvedUrl : "") << "\"}";
        } else {
            if (shown == 0)
                std::cout << "\n" << std::left << std::setw(24) << "segment"
                          << std::setw(16) << "size (WxH)" << std::setw(12)
                          << "cache" << "tifxyz url\n";
            std::ostringstream size;
            size << seg.width.value_or(0) << "x" << seg.height.value_or(0);
            std::cout << std::left << std::setw(24) << seg.id << std::setw(16)
                      << size.str() << std::setw(12) << cacheStateName(state)
                      << trimTo(art ? art->resolvedUrl : "", 70) << "\n";
        }
        ++shown;
    }
    if (json)
        std::cout << "]\n";
    else
        std::cout << "\n(" << shown << " tifxyz segments; cache root "
                  << cacheRoot.string() << ")\n";
    return 0;
}

int cmdPull(
    const OpenDataManifest& manifest, const std::string& sampleId,
    std::vector<std::string> segmentIds, int smallest,
    const fs::path& cacheRoot, bool force)
{
    const OpenDataSample* sample = findSampleById(manifest, sampleId);
    if (!sample) {
        std::cerr << "error: sample '" << sampleId << "' not in the manifest\n";
        return 2;
    }

    // Pick targets.
    std::vector<const OpenDataSegment*> targets;
    if (!segmentIds.empty()) {
        for (const auto& id : segmentIds) {
            const OpenDataSegment* found = nullptr;
            for (const auto& seg : sample->segments)
                if (seg.id == id || seg.longId == id)
                    found = &seg;
            if (!found || !found->hasTifxyz()) {
                std::cerr << "error: segment '" << id
                          << "' not found (or has no tifxyz) in " << sampleId
                          << "\n";
                return 2;
            }
            targets.push_back(found);
        }
    } else if (smallest > 0) {
        std::vector<const OpenDataSegment*> all;
        for (const auto& seg : sample->segments)
            if (seg.hasTifxyz())
                all.push_back(&seg);
        std::sort(all.begin(), all.end(),
                  [](const OpenDataSegment* a, const OpenDataSegment* b) {
                      return segmentCells(*a) < segmentCells(*b);
                  });
        for (int i = 0; i < smallest && i < (int)all.size(); ++i)
            targets.push_back(all[i]);
    } else {
        std::cerr << "error: name segment ids or pass --smallest=N\n";
        return 1;
    }

    // Reconcile writes placeholder dirs (meta.json synthesized from the
    // manifest + a materialization recipe) for every supported segment;
    // downloads only happen for the segments we materialize below.
    auto pkg = VolumePkg::newEmpty(
        vc::project::LoadOptions{cacheRoot, false, false});
    auto progress = [](const OpenDataSampleDownloadProgress& p) {
        if (!p.status.empty())
            std::cout << "  [reconcile] " << p.status << "\n";
    };
    const auto reconcile =
        reconcileOpenDataSampleSegments(*pkg, *sample, cacheRoot, progress, force);
    std::cout << "reconciled: " << reconcile.supportedTifxyzSegments
              << " supported tifxyz segments, "
              << reconcile.cachedTifxyzSegments << " already cached\n";
    for (const auto& msg : reconcile.messages)
        std::cout << "  [reconcile] " << msg << "\n";

    int failures = 0;
    for (const auto* seg : targets) {
        const fs::path dir =
            openDataCanonicalSegmentCacheDirectory(cacheRoot, *sample, *seg);
        std::cout << "\npull " << seg->id << " ("
                  << seg->width.value_or(0) << "x" << seg->height.value_or(0)
                  << ")\n  -> " << dir.string() << "\n";
        if (!fs::exists(dir)) {
            std::cerr << "  error: no placeholder was created (unsupported "
                         "representation?)\n";
            ++failures;
            continue;
        }
        const auto result = materializeOpenDataSegment(dir);
        if (result.success) {
            std::cout << (result.alreadyMaterialized
                              ? "  already materialized\n"
                              : "  downloaded\n");
        } else {
            std::cerr << "  error: " << result.message << "\n";
            ++failures;
        }
    }

    std::cout << "\n" << (targets.size() - failures) << "/" << targets.size()
              << " segments ready under " << cacheRoot.string() << "\n";
    return failures ? 2 : 0;
}

}  // namespace

int main(int argc, char* argv[])
{
    // Unbuffered: a network CLI should stream its progress even when piped.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    if (argc < 2)
        return usage(argv[0]);

    const std::string command = argv[1];
    std::string manifestArg;
    std::string sampleId;
    std::vector<std::string> segmentIds;
    fs::path cacheRoot = fs::absolute("vc_opendata_cache");
    bool json = false;
    bool force = false;
    int smallest = 0;

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--manifest=", 0) == 0) {
            manifestArg = a.substr(11);
        } else if (a.rfind("--cache=", 0) == 0) {
            cacheRoot = fs::absolute(a.substr(8));
        } else if (a.rfind("--smallest=", 0) == 0) {
            smallest = std::stoi(a.substr(11));
        } else if (a == "--json") {
            json = true;
        } else if (a == "--force") {
            force = true;
        } else if (a.rfind("--", 0) == 0) {
            std::cerr << "error: unknown option '" << a << "'\n";
            return 1;
        } else if (sampleId.empty()) {
            sampleId = a;
        } else {
            segmentIds.push_back(a);
        }
    }

    try {
        const OpenDataManifest manifest = loadManifest(manifestArg);
        if (manifest.samples.empty()) {
            std::cerr << "error: manifest is empty or could not be parsed\n";
            return 2;
        }
        if (command == "list")
            return cmdList(manifest, json);
        if (command == "segments") {
            if (sampleId.empty())
                return usage(argv[0]);
            return cmdSegments(manifest, sampleId, json, cacheRoot);
        }
        if (command == "pull") {
            if (sampleId.empty())
                return usage(argv[0]);
            return cmdPull(manifest, sampleId, std::move(segmentIds), smallest,
                           cacheRoot, force);
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }

    std::cerr << "error: unknown command '" << command << "'\n";
    return usage(argv[0]);
}
