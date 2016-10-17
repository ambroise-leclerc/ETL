/// @file functional.h
/// @date 14/03/2014 17:02:55
/// @author Ambroise Leclerc
/// @brief Function objects.
//
// Copyright (c) 2014, Ambroise Leclerc
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
#pragma once
#include <libstd/include/type_traits>
#include <libstd/include/utility>

namespace ETLSTD {

namespace etlHelper { /*
	template<typename Base, typename T, typename Derived, typename... Args>
	auto invoke(T Base::*pmf, Derived&& ref, Args&&... args) noexcept(noexcept((std::forward<Derived>(ref).*pmf)(std::forward<Args>(args)...)))
	-> std::enable_if_t<std::is_function_v<T> && std::is_base_of_v<Base, std::decay_t<Derived>>, decltype((std::forward<Derived>(ref).*pmf)(std::forward<Args>(args)...))> {
		return (std::forward<Derived>(ref).*pmf)(std::forward<Args>(args)...);
	}
	
	template<typename Base, typename T, typename RefWrap, typename... Args>
	auto invoke(T Base::*pmf, RefWrap&& ref, Args&&... args) noexcept(noexcept((ref.get().*pmf)(std::forward<Args>(args)...)))
	-> std::enable_if_t<std::is_function_v<T> && is_reference_wrapper_v<std::decay_t<RefWrap>>, decltype((ref.get().*pmf)(std::forward<Args>(args)...))> {
		return (ref.get().*pmf)(std::forward<Args>(args)...);
	}
	
	template<typename Base, typename T, typename Pointer, typename... Args>
	auto invoke(T Base::*pmf, Pointer&& ptr, Args&&... args) noexcept(noexcept(((*std::forward<Pointer>(ptr)).*pmf)(std::forward<Args>(args)...)))
	-> std::enable_if_t<std::is_function_v<T> && !is_reference_wrapper_v<std::decay_t<Pointer>> && !std::is_base_of_v<Base, std::decay_t<Pointer>>, decltype(((*std::forward<Pointer>(ptr)).*pmf)(std::forward<Args>(args)...))> {
		return ((*std::forward<Pointer>(ptr)).*pmf)(std::forward<Args>(args)...);
	}
	
	template<typename Base, typename T, typename Derived>
	auto invoke(T Base::*pmd, Derived&& ref) noexcept(noexcept(std::forward<Derived>(ref).*pmd))
	-> std::enable_if_t<!std::is_function_v<T> && std::is_base_of_v<Base, std::decay_t<Derived>>, decltype(std::forward<Derived>(ref).*pmd)> {
		return std::forward<Derived>(ref).*pmd;
	}
	
	template<typename Base, typename T, typename RefWrap>
	auto invoke(T Base::*pmd, RefWrap&& ref) noexcept(noexcept(ref.get().*pmd))
	-> std::enable_if_t<!std::is_function_v<T> && is_reference_wrapper_v<std::decay_t<RefWrap>>, decltype(ref.get().*pmd)> {
		return ref.get().*pmd;
	}
	
	template<typename Base, typename T, typename Pointer>
	auto invoke(T Base::*pmd, Pointer&& ptr) noexcept(noexcept((*std::forward<Pointer>(ptr)).*pmd))
	-> std::enable_if_t<!std::is_function_v<T> && !is_reference_wrapper_v<std::decay_t<Pointer>> && !std::is_base_of_v<Base, std::decay_t<Pointer>>, decltype((*std::forward<Pointer>(ptr)).*pmd)> {
		return (*std::forward<Pointer>(ptr)).*pmd;
	}
	
	template<typename F, typename... Args>
	auto invoke(F&& f, Args&&... args) noexcept(noexcept(std::forward<F>(f)(std::forward<Args>(args)...)))
	-> std::enable_if_t<!std::is_member_pointer_v<std::decay_t<F>>, decltype(std::forward<F>(f)(std::forward<Args>(args)...))> {
		return std::forward<F>(f)(std::forward<Args>(args)...);
	}
	*/
	template <class F, class... Args> inline auto invoke(F&& f, Args&&... args) -> decltype(forward<F>(f)(forward<Args>(args)...)) {
		return forward<F>(f)(forward<Args>(args)...);
	}
	
	template <class Base, class T, class Derived> inline auto invoke(T Base::*pmd, Derived&& ref) -> decltype(forward<Derived>(ref).*pmd) {
		return forward<Derived>(ref).*pmd;
	}
	
	template <class PMD, class Pointer> inline auto invoke(PMD&& pmd, Pointer&& ptr) ->	decltype((*forward<Pointer>(ptr)).*forward<PMD>(pmd)) {
		return (*forward<Pointer>(ptr)).*forward<PMD>(pmd);
	}
	
	template <class Base, class T, class Derived, class... Args> inline auto invoke(T Base::*pmf, Derived&& ref, Args&&... args) ->	decltype((forward<Derived>(ref).*pmf)(forward<Args>(args)...)) {
		return (forward<Derived>(ref).*pmf)(forward<Args>(args)...);
	}
	
	template <class PMF, class Pointer, class... Args> inline auto INVOKE(PMF&& pmf, Pointer&& ptr, Args&&... args) -> decltype(((*forward<Pointer>(ptr)).*forward<PMF>(pmf))(forward<Args>(args)...)) {
		return ((*forward<Pointer>(ptr)).*forward<PMF>(pmf))(forward<Args>(args)...);
	}

	template<typename, typename = void> struct result_of {};
	template<typename F, typename...Args> struct result_of<F(Args...), decltype(void(invoke(declval<F>(), declval<Args>()...)))> {
		using type = decltype(invoke(declval<F>(), declval<Args>()...));
};
} // namespace etlHelper

    
/// Invoke the Callable object f with the parameters args.
/// @param[in] f Callable to be invoked
/// @param[in] args arguments to pass to f
template <typename F, typename... ArgTypes>
auto invoke(F&& f, ArgTypes&&... args) noexcept(noexcept(etlHelper::invoke(forward<F>(f), forward<ArgTypes>(args)...))) {
    return etlHelper::invoke(forward<F>(f), forward<ArgTypes>(args)...);
}

/// Deduces the return type of a function call expression at compile time.    
template<typename F> struct result_of {};
template<typename F, typename... ArgTypes> struct result_of<F(ArgTypes...)> : etlHelper::result_of<F(ArgTypes...)> {};


} // namespace ETLSTD  

