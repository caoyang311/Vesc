---
name: "c-safety-standard"
description: "Generates, modifies, and reviews safety-related C code using MISRA C:2012, ISO 26262 static-analysis practices, and Doxygen function headers. Invoke for any C coding or review task."
---

# C Safety Coding Standard

## Purpose

Apply this skill whenever generating, modifying, refactoring, or reviewing C source or header files in this workspace.

The objective is to produce deterministic, analyzable embedded C code aligned with:

- MISRA C:2012 rules and directives.
- ISO 26262 expectations for static analyzability, defensive design, traceability, and controlled deviations.
- Doxygen documentation for every function declaration and definition.

This skill does not claim formal MISRA or ISO 26262 compliance by itself. Compliance requires qualified tools, project-specific rule classification, deviation records, reviews, traceability, and safety lifecycle evidence.

## Mandatory Workflow

1. Read the affected source, headers, configuration, and call sites before changing code.
2. Identify safety boundaries, input ranges, ownership, concurrency, ISR context, and error behavior.
3. Make the smallest change that satisfies the requirement.
4. Check declarations, definitions, integer conversions, array bounds, pointer validity, control flow, and initialization.
5. Add or update Doxygen function headers for every affected function declaration and definition.
6. Run available compiler diagnostics and static analysis.
7. Report unresolved warnings, assumptions, and required MISRA deviations. Never silently suppress a violation.

## C Language Rules

### Types and Conversions

- Use fixed-width integer types from `<stdint.h>` for persisted data, interfaces, hardware values, counters, and protocol fields.
- Use `bool` from `<stdbool.h>` for logical state.
- Use typedefs for domain identifiers and externally visible interface types.
- Do not use plain `char`, `short`, `int`, `long`, signedness-dependent bit fields, or implementation-sized enums where width matters.
- Do not mix signed and unsigned operands without an explicit, justified conversion.
- Suffix integer constants with the correct type, such as `0U`, `1UL`, or `INT32_C()`.
- Check range before narrowing conversions. Make intentional conversions explicit.
- Avoid floating-point equality comparisons. Define tolerances where comparison is required.

### Initialization and Lifetime

- Initialize every automatic variable before use.
- Explicitly initialize structures and arrays.
- Keep object scope and lifetime as narrow as possible.
- Declare file-private objects and functions as `static`.
- Use `const` for read-only data, pointer targets, parameters, and configuration tables where applicable.
- Do not use dynamic memory allocation in embedded production code unless explicitly approved by the safety architecture.
- Do not use variable-length arrays.

### Functions

- Functions shall have a single clear responsibility and bounded execution time.
- Declare parameterless functions with `(void)`.
- Provide a prototype before use; no implicit declarations are permitted.
- Keep the declaration and definition signatures identical.
- Validate data at external boundaries. Avoid redundant checks for internally guaranteed invariants.
- Return explicit status values when an operation can fail.
- Check returned status unless an intentional discard is documented.
- Do not use recursion.
- Avoid hidden state and unnecessary global variables. Use explicit context objects for reusable algorithms.
- Pointer parameters shall document nullability, direction, size, and ownership.

### Control Flow

- Every `if`, `else`, `for`, `while`, `do`, and `switch` body shall use braces.
- Every non-empty `switch` shall contain a `default` branch or a documented justified deviation.
- Each `case` shall terminate explicitly; intentional fall-through requires a clear annotation accepted by the static-analysis configuration.
- Avoid `goto`. If cleanup requires it, document and review the deviation.
- Avoid multiple `break`/`continue` paths that obscure flow.
- Conditions shall be essentially Boolean; do not rely on implicit integer-to-Boolean conversion.
- Loops shall have demonstrably bounded termination behavior.
- Do not place assignments inside conditions.

### Pointers, Arrays, and Memory

- Check external pointer arguments before dereference unless the interface contract guarantees non-null and documents that contract.
- Do not perform pointer arithmetic outside the bounds of the same array object.
- Use array capacity and valid length as separate values.
- Validate indices before array access.
- Use `sizeof(array) / sizeof(array[0])` only where the operand is an actual array, not a pointer.
- Avoid casts that remove `const` or `volatile`.
- Do not cast between object pointers and function pointers.
- Avoid type punning, incompatible pointer casts, and strict-aliasing violations.
- Use `memcpy` only with validated sizes and compatible object representations.

### Expressions and Side Effects

- Avoid expressions whose result depends on evaluation order.
- Use one meaningful side effect per statement.
- Do not modify an object more than once between sequence points.
- Parenthesize mixed operators when precedence is not immediately obvious.
- Avoid macros that evaluate arguments multiple times.
- Prefer typed functions or `static inline` functions over function-like macros.
- Parenthesize all macro parameters and the complete macro replacement expression.

### Preprocessor and Headers

- Every header shall have a unique include guard.
- Headers shall be self-contained and include the headers required by their public declarations.
- Public headers shall expose only necessary types and interfaces.
- Hardware and vendor headers shall not leak into platform-independent public interfaces.
- Do not use `#define` to replace typed constants where an enum, `static const`, or typed configuration is suitable.
- Avoid conditional compilation inside function bodies.
- Do not redefine reserved identifiers or standard library names.

### Interrupts and Concurrency

- ISR code shall be short, bounded, non-blocking, and deterministic.
- ISR code shall not call delays, allocate memory, parse protocols, or execute ordinary application tasks.
- Shared ISR/task objects shall use `volatile` where hardware or asynchronous modification requires it; `volatile` is not a synchronization primitive.
- Protect multi-access shared data using a justified atomic operation, critical section, snapshot, or lock-free protocol.
- Keep critical sections minimal and restore the previous interrupt state.
- Document single-writer/multiple-reader ownership and update rules.

### Hardware Access

- Application and algorithm modules shall not directly access peripheral registers, HAL handles, ports, or pins.
- Hardware access shall be isolated behind Platform interfaces.
- Register accesses shall use vendor-provided volatile definitions or reviewed low-level abstractions.
- Validate logical channel IDs and configuration-table indices before hardware access.
- Configuration counts shall be checked against actual table sizes with compile-time assertions where possible.

## ISO 26262-Oriented Static Analysis Practices

For safety-related code:

- Enable the compiler's strongest practical warning set and treat project-approved warnings as errors.
- Run a MISRA C:2012-capable static analyzer for all production C files.
- Classify every finding as fixed, false positive, or approved deviation.
- A deviation shall record rule/directive, location, rationale, risk, mitigation, approver, and scope.
- Do not suppress warnings globally when a local, reviewed deviation is sufficient.
- Require no unresolved high-severity findings before integration.
- Maintain traceability from safety/software requirements to implementation and tests.
- Verify boundary values, invalid identifiers, null pointers, integer limits, timing assumptions, and error paths.
- Record assumptions about compiler, target widths, endianness, atomicity, interrupt priority, and hardware behavior.
- Review generated code and third-party code separately; define which MISRA rules apply and where deviations are accepted.

## Doxygen Function Header Standard

Every function declaration in a public or private header and every function definition in a C file shall have a Doxygen header immediately before it.

Use this format:

```c
/**
 * @brief One-sentence description of the function.
 *
 * Detailed behavior, preconditions, side effects, timing context, and
 * concurrency constraints when relevant.
 *
 * @param[in] input Description, valid range, units, and nullability.
 * @param[out] output Description, size, ownership, and nullability.
 * @param[in,out] context Description of data read and modified.
 *
 * @return Description of every possible return value.
 *
 * @pre Required state before calling.
 * @post Guaranteed state after successful completion.
 * @note Important implementation or integration constraint.
 */
```

Rules:

- Use `@param[in]`, `@param[out]`, or `@param[in,out]` for every parameter.
- Describe units and valid ranges.
- State whether pointers may be `NULL`.
- Include `@return` for every non-void function and enumerate status meanings.
- Omit empty tags for simple functions, but always keep `@brief`.
- For ISR callbacks, document interrupt source, shared data, and execution restrictions.
- For periodic tasks, document period, execution context, and blocking prohibition.
- Keep declaration and definition documentation consistent. The public declaration contains the interface contract; the definition may add implementation details.
- Use the spelling `Doxygen`, not `Doxyen`, in generated documentation.

Example:

```c
/**
 * @brief Writes a logical level to a configured DIO channel.
 *
 * @param[in] channel_id Logical channel identifier. The value shall be less
 *                       than DIO_CONFIGURED_CHANNEL_COUNT.
 * @param[in] level Requested level. STD_HIGH writes high; all other values
 *                  are treated as STD_LOW.
 *
 * @return None.
 *
 * @note Invalid channel identifiers are ignored and cause no hardware access.
 */
void Dio_WriteChannel(Dio_ChannelType channel_id, Dio_LevelType level);
```

## Review Checklist

Before completing a C task, verify:

- [ ] All changed functions have Doxygen headers.
- [ ] Public headers are self-contained and do not leak platform details.
- [ ] Declarations and definitions match.
- [ ] Variables are initialized before use.
- [ ] Signedness and width conversions are explicit and safe.
- [ ] Array indexes and channel IDs are checked.
- [ ] Pointer contracts and null behavior are documented.
- [ ] Return values are checked or intentionally documented as discarded.
- [ ] No recursion, dynamic allocation, unbounded loop, or hidden blocking exists.
- [ ] Switch statements, fall-through, and default behavior are explicit.
- [ ] ISR/task shared data has a documented synchronization strategy.
- [ ] Hardware access stays in the Platform layer.
- [ ] Static-analysis findings and deviations are reported.
- [ ] Available diagnostics, build, tests, and analyzers have been run.

## Output Requirements

When applying this skill, summarize:

1. Rules applied to the change.
2. Static-analysis or compiler checks performed.
3. Remaining findings and their severity.
4. Any required MISRA deviation and rationale.
5. Tests or evidence still required for ISO 26262 lifecycle compliance.
