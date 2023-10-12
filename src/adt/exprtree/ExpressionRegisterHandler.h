#pragma once

#include <vector>
#include <memory>
#include <initializer_list>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle expression registers.
 *
 * The register handler supports accessing register values by their indices,
 *  where the registers are stored as pointers to variables of interest inside
 *  the software using the ExpressionRegisterHandler
 *
 * @tparam NumericType The numeric type of the registers.
 * @see RegUnivarExprTreeNode
 */
template <typename NumericType>
class ExpressionRegisterHandler{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The array of Expression Registers (ER) pointers
     * @see ExpressionRegisterHandler::numRegisters
     */
    NumericType const ** er;
    /**
     * @brief The number of registers in the array of expression registers
     * @see ExpressionRegisterHandler::er
     */
    size_t numRegisters = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a expression register handler with given registers as
     *  vector
     * @param registers The registers to which the handler must be linked
     */
    ExpressionRegisterHandler(
        std::vector<NumericType const *> registers
    ) :
        numRegisters(registers.size())
    {
        er = new NumericType const * [numRegisters];
        for(size_t i = 0 ; i < numRegisters ; ++i) er[i] = registers[i];
    }
    /**
     * @brief Build a expression register handler with given registers as
     *  initializer list
     * @param registers The registers to which the handler must be linked
     */
    ExpressionRegisterHandler(
        std::initializer_list<NumericType const *> registers
    ) :
        numRegisters(registers.size())
    {
        er = new NumericType const * [numRegisters];
        for(size_t i = 0 ; i < numRegisters ; ++i) er[i] = registers[i];
    }
    virtual ~ExpressionRegisterHandler(){
        delete[] er; // Delete the array of pointers, not the pointers
    }

    // *** GETTERs and SETTERs *** //
    // *************************** //
    /**
     * @brief Obtain the pointer to the i-th register
     * @param i The index of the register which pointer shall be obtained
     * @return A pointer to the i-th register as a constant
     * @see ExpressionRegisterHandler::er
     * @see ExpressionRegisterHandler::getRegByVal
     */
    inline NumericType const * getReg(size_t const i) {return er[i];}
    /**
     * @brief Obtain the i-th register by value
     * @param i The index of the register which value shall be obtained
     * @return The value of the i-th register
     * @see ExpressionRegisterHandler::er
     * @see ExpressionRegisterHandler::getReg
     */
    inline NumericType const getRegByVal(size_t const i) {return *getReg(i);}
    /**
     * @brief Obtain the number of handled registers
     * @return The number of handled registers
     * @see ExpressionRegisterHandler::numRegisters
     */
    inline size_t getNumRegisters() {return numRegisters;}
};