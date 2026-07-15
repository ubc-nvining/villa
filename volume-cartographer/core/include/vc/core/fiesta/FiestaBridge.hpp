#pragma once

// FiestaBridge — runtime binding to the ScrollFiesta library.
//
// ScrollFiesta is loaded Carmack-style: dlopen()/LoadLibrary() the shared
// library, resolve the single exported symbol "sf_get_api", and call
// everything through the returned function-pointer table. VC3D never links
// the library — the contract is pure C ABI, so a scrollfiesta.dll built with
// any compiler drops in next to the executable, and its absence just means
// the feature is unavailable (never fatal).
//
// Search order: directory of the current executable -> SCROLLFIESTA_DLL
// environment variable (full path) -> the platform's default library search.

#include <scrollfiesta.h>

#include "vc/core/util/TriMeshBridge.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace vc::fiesta
{

class FiestaError : public std::runtime_error
{
public:
    FiestaError(sf_status status, const std::string& what)
        : std::runtime_error(what), _status(status)
    {
    }
    sf_status status() const { return _status; }

private:
    sf_status _status;
};

class FiestaCancelled : public FiestaError
{
public:
    FiestaCancelled() : FiestaError(SF_CANCELLED, "operation cancelled") {}
};

// Progress callback: fraction in [0,1] + a short stable stage id.
// Return false to request cancellation.
using ProgressFn = std::function<bool(float fraction, const char* stage)>;

// Process-wide loader + table. Loading happens once, lazily, on first use.
class FiestaRuntime
{
public:
    static FiestaRuntime& instance();

    bool available() const { return _api != nullptr; }
    // Human-readable reason when !available() ("scrollfiesta.dll not found
    // next to ...", "ABI N required, library provides M", ...).
    const std::string& unavailableReason() const { return _reason; }

    const sf_api* api() const { return _api; }
    // Like api(), but throws FiestaError when the library is unavailable.
    const sf_api* require() const;

    std::string versionString() const;

    FiestaRuntime(const FiestaRuntime&) = delete;
    FiestaRuntime& operator=(const FiestaRuntime&) = delete;

private:
    FiestaRuntime();
    const sf_api* _api = nullptr;
    std::string _reason;
    std::string _loadedPath;
};

// RAII over a library-owned sf_mesh (freed through the table).
class SfMeshOwner
{
public:
    SfMeshOwner() = default;
    explicit SfMeshOwner(const sf_api* api) : _api(api) {}
    ~SfMeshOwner() { reset(); }
    SfMeshOwner(SfMeshOwner&& o) noexcept : _api(o._api), _mesh(o._mesh)
    {
        o._mesh = sf_mesh{};
    }
    SfMeshOwner& operator=(SfMeshOwner&& o) noexcept
    {
        if (this != &o) {
            reset();
            _api = o._api;
            _mesh = o._mesh;
            o._mesh = sf_mesh{};
        }
        return *this;
    }
    SfMeshOwner(const SfMeshOwner&) = delete;
    SfMeshOwner& operator=(const SfMeshOwner&) = delete;

    sf_mesh* out() { return &_mesh; }  // pass to an op as its output slot
    const sf_mesh& get() const { return _mesh; }
    void reset()
    {
        if (_api && _mesh.vertices)
            _api->mesh_free(&_mesh);
        _mesh = sf_mesh{};
    }

private:
    const sf_api* _api = nullptr;
    sf_mesh _mesh{};
};

// Same for an sf_mesh_list.
class SfMeshListOwner
{
public:
    SfMeshListOwner() = default;
    explicit SfMeshListOwner(const sf_api* api) : _api(api) {}
    ~SfMeshListOwner() { reset(); }
    SfMeshListOwner(const SfMeshListOwner&) = delete;
    SfMeshListOwner& operator=(const SfMeshListOwner&) = delete;

    sf_mesh_list* out() { return &_list; }
    const sf_mesh_list& get() const { return _list; }
    void reset()
    {
        if (_api && _list.items)
            _api->mesh_list_free(&_list);
        _list = sf_mesh_list{};
    }

private:
    const sf_api* _api = nullptr;
    sf_mesh_list _list{};
};

// Non-owning sf_mesh view over a TriMesh (valid while the TriMesh lives;
// ScrollFiesta never modifies or retains caller inputs).
sf_mesh viewOf(const core::util::TriMesh& mesh);

// Deep-copy an sf_mesh into a TriMesh (+ its provenance vmap, empty if none).
core::util::TriMesh toTriMesh(const sf_mesh& m);
std::vector<int32_t> vmapOf(const sf_mesh& m);

// Wire a ProgressFn into sf_common_opts. The trampoline context must outlive
// the ScrollFiesta call; check rethrow() afterwards to propagate exceptions
// the callback threw (they are converted to a cancel request inside the C
// boundary — nothing ever throws across it).
struct ProgressTrampoline {
    explicit ProgressTrampoline(ProgressFn fn) : fn(std::move(fn)) {}
    void attach(sf_common_opts& opts);
    void rethrow() const;

    ProgressFn fn;
    std::exception_ptr pending;
};

// Map a non-OK status to the matching exception (FiestaCancelled for
// SF_CANCELLED, FiestaError otherwise). No-op on SF_OK.
void throwOnError(const sf_api* api, sf_status status, const char* opName);

}  // namespace vc::fiesta
