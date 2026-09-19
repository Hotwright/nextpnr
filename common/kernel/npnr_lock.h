/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Claire Xenia Wolf <claire@yosyshq.com>
 *  Copyright (C) 2018  Serge Bazanski <q3k@q3k.org>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef NPNR_LOCK_H
#define NPNR_LOCK_H

#include <mutex>

#include "nextpnr_namespaces.h"

NEXTPNR_NAMESPACE_BEGIN

// Scope guards for the Context lock.
//
// libc++ defines std::lock_guard and std::unique_lock only when it is built
// with thread support: on a no-threads target such as wasm32-wasi, <mutex>
// compiles them out entirely and `std::lock_guard` is not merely unusable but
// undeclared. The placer and the routers take the Context lock unconditionally,
// so without these aliases they do not compile there at all.
//
// Substituting a do-nothing type is exact rather than approximate. Under
// NPNR_DISABLE_THREADS - which is what a no-threads target gets - BaseCtx::lock()
// and BaseCtx::unlock() have empty bodies, so the real std:: guards already
// acquire nothing; these stand-ins acquire the same nothing.
#ifndef NPNR_DISABLE_THREADS

template <typename T> using ScopeLock = std::lock_guard<T>;
template <typename T> using ScopeUniqueLock = std::unique_lock<T>;

#else

template <typename T> struct ScopeLock
{
    explicit ScopeLock(T &) {}
    ScopeLock(const ScopeLock &) = delete;
    ScopeLock &operator=(const ScopeLock &) = delete;
};

template <typename T> struct ScopeUniqueLock
{
    explicit ScopeUniqueLock(T &) {}
    ScopeUniqueLock(const ScopeUniqueLock &) = delete;
    ScopeUniqueLock &operator=(const ScopeUniqueLock &) = delete;
    void lock() {}
    void unlock() {}
};

#endif

NEXTPNR_NAMESPACE_END

#endif // NPNR_LOCK_H
