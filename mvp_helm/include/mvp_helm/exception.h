#pragma once

#include <exception>
#include <utility>
#include <string>
#include <stdexcept>

namespace helm {

    class HelmException : public std::exception {
    protected:
        /** Error message.
         */
        std::runtime_error M;
    public:
        /** Constructor (C strings).
         *  @param message C-style string error message.
         *                 The string contents are copied upon construction.
         *                 Hence, responsibility for deleting the char* lies
         *                 with the caller.
         */
        explicit HelmException(const char* message)
            : M(message) {}

        /** Constructor (C++ STL strings).
         *  @param message The error message.
         */
        explicit HelmException(const std::string&  message)
            : M(message) {}

        /** Destructor.
         * Virtual to allow for subclassing.
         */
        ~HelmException() noexcept override = default;

        /** Returns a pointer to the (constant) error description.
         *  @return A pointer to a const char*. The underlying memory
         *          is in posession of the Exception object. Callers must
         *          not attempt to free the memory.
         */
        const char* what() const noexcept override {
            return M.what();
        }

    };
    
} // namespace helm