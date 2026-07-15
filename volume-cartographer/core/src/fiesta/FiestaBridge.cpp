#include "vc/core/fiesta/FiestaBridge.hpp"

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <sstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

namespace vc::fiesta
{

namespace
{

#ifdef _WIN32
constexpr const char* kLibNames[] = {"scrollfiesta.dll"};
#elif defined(__APPLE__)
constexpr const char* kLibNames[] = {"libscrollfiesta.dylib"};
#else
constexpr const char* kLibNames[] = {"libscrollfiesta.so", "libscrollfiesta.so.0"};
#endif

std::filesystem::path executableDir()
{
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH)
        return {};
    return std::filesystem::path(buf).parent_path();
#elif defined(__APPLE__)
    char buf[4096];
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0)
        return {};
    return std::filesystem::canonical(buf).parent_path();
#else
    std::error_code ec;
    auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec)
        return {};
    return p.parent_path();
#endif
}

void* openLibrary(const std::filesystem::path& path)
{
#ifdef _WIN32
    return (void*)LoadLibraryW(path.wstring().c_str());
#else
    return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
}

void* openLibraryByName(const char* name)
{
#ifdef _WIN32
    return (void*)LoadLibraryA(name);
#else
    return dlopen(name, RTLD_NOW | RTLD_LOCAL);
#endif
}

void* symbolOf(void* handle, const char* name)
{
#ifdef _WIN32
    return (void*)GetProcAddress((HMODULE)handle, name);
#else
    return dlsym(handle, name);
#endif
}

}  // namespace

FiestaRuntime::FiestaRuntime()
{
    void* handle = nullptr;
    std::ostringstream tried;

    // 1. Next to the executable.
    const auto exe_dir = executableDir();
    if (!exe_dir.empty()) {
        for (const char* name : kLibNames) {
            const auto candidate = exe_dir / name;
            if (std::filesystem::exists(candidate)) {
                handle = openLibrary(candidate);
                if (handle) {
                    _loadedPath = candidate.string();
                    break;
                }
                tried << candidate.string() << " (failed to load); ";
            } else {
                tried << candidate.string() << " (not found); ";
            }
        }
    }

    // 2. SCROLLFIESTA_DLL environment override (full path).
    if (!handle) {
        if (const char* env = std::getenv("SCROLLFIESTA_DLL")) {
            handle = openLibrary(env);
            if (handle)
                _loadedPath = env;
            else
                tried << env << " (SCROLLFIESTA_DLL, failed to load); ";
        }
    }

    // 3. Default library search.
    if (!handle) {
        for (const char* name : kLibNames) {
            handle = openLibraryByName(name);
            if (handle) {
                _loadedPath = name;
                break;
            }
        }
    }

    if (!handle) {
        _reason = "ScrollFiesta library not found (tried: " + tried.str() +
                  "default search paths)";
        return;
    }

    auto get_api = (sf_get_api_fn)symbolOf(handle, "sf_get_api");
    if (!get_api) {
        _reason = _loadedPath + " does not export sf_get_api";
        return;
    }
    const sf_api* api = get_api(SCROLLFIESTA_ABI_VERSION);
    if (!api) {
        _reason = _loadedPath + " does not implement ABI " +
                  std::to_string(SCROLLFIESTA_ABI_VERSION) +
                  " (rebuild VC3D or swap in a matching scrollfiesta library)";
        return;
    }
    if (api->struct_size < sizeof(sf_api)) {
        // Older library within the same ABI would be a contract violation
        // (the table is append-only per ABI version); refuse rather than
        // call off the end of its table.
        _reason = _loadedPath + " has a smaller sf_api table than this build expects";
        return;
    }
    _api = api;
}

FiestaRuntime& FiestaRuntime::instance()
{
    static FiestaRuntime runtime;
    return runtime;
}

const sf_api* FiestaRuntime::require() const
{
    if (!_api)
        throw FiestaError(SF_ERROR_UNSUPPORTED, _reason);
    return _api;
}

std::string FiestaRuntime::versionString() const
{
    if (!_api)
        return {};
    std::string v = _api->version_string();
    if (!_loadedPath.empty())
        v += " (" + _loadedPath + ")";
    return v;
}

sf_mesh viewOf(const core::util::TriMesh& mesh)
{
    static_assert(sizeof(cv::Vec3f) == 3 * sizeof(float));
    static_assert(sizeof(cv::Vec3i) == 3 * sizeof(int32_t));
    sf_mesh m{};
    m.vertices = const_cast<float*>(
        reinterpret_cast<const float*>(mesh.vertices.data()));
    m.faces = const_cast<int32_t*>(
        reinterpret_cast<const int32_t*>(mesh.faces.data()));
    if (!mesh.normals.empty())
        m.vertex_normals = const_cast<float*>(
            reinterpret_cast<const float*>(mesh.normals.data()));
    m.n_vertices = mesh.vertices.size();
    m.n_faces = mesh.faces.size();
    return m;
}

core::util::TriMesh toTriMesh(const sf_mesh& m)
{
    core::util::TriMesh mesh;
    mesh.vertices.resize(m.n_vertices);
    std::memcpy(
        mesh.vertices.data(), m.vertices, m.n_vertices * 3 * sizeof(float));
    mesh.faces.resize(m.n_faces);
    std::memcpy(mesh.faces.data(), m.faces, m.n_faces * 3 * sizeof(int32_t));
    if (m.vertex_normals) {
        mesh.normals.resize(m.n_vertices);
        std::memcpy(
            mesh.normals.data(), m.vertex_normals,
            m.n_vertices * 3 * sizeof(float));
    }
    return mesh;
}

std::vector<int32_t> vmapOf(const sf_mesh& m)
{
    if (!m.vmap)
        return {};
    return std::vector<int32_t>(m.vmap, m.vmap + m.n_vertices);
}

namespace
{

int progressThunk(void* user, const char* stage, double fraction)
{
    auto* tramp = static_cast<ProgressTrampoline*>(user);
    if (!tramp->fn)
        return 0;
    try {
        return tramp->fn(static_cast<float>(fraction), stage) ? 0 : 1;
    } catch (...) {
        // Nothing may throw across the C boundary: convert to a cancel
        // request and rethrow after the call returns.
        tramp->pending = std::current_exception();
        return 1;
    }
}

}  // namespace

void ProgressTrampoline::attach(sf_common_opts& opts)
{
    if (fn) {
        opts.progress = progressThunk;
        opts.progress_user = this;
    }
}

void ProgressTrampoline::rethrow() const
{
    if (pending)
        std::rethrow_exception(pending);
}

void throwOnError(const sf_api* api, sf_status status, const char* opName)
{
    if (status == SF_OK)
        return;
    if (status == SF_CANCELLED)
        throw FiestaCancelled();
    std::string what = std::string(opName) + ": " +
                       (api ? api->status_str(status) : "scrollfiesta error");
    throw FiestaError(status, what);
}

}  // namespace vc::fiesta
