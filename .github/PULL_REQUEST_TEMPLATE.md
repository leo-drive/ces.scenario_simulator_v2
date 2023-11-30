# PLEASE REMOVE IT AND WRITE TITLE OF YOUR PULL REQUEST

## Description 📚

### Background

> Please delete this sentence and explain how and why this pull request was sent.

### Purpose

> Please delete this sentence and describe why you sent this pull request.

### Specification

> Please delete this sentence and describe the specifications of any features added or changed in this pull request.

### Reference to a key algorithm

> Please delete this sentence and describe the key algorithm used in the implementation. If there is any novelty, please mention it.

## Destructive Changes 💥

> If this pull request changes the behavior of public functions in the traffic_simulator::API class or scenarios written in OpenSCENARIO, please include a description of the changes and a migration guide and send this pull request with a `bump major` label on this pull request, describing the changes and the migration guide.

> Otherwise, please remove the "Destructive Changes" section and make sure this pull request is labeled `bump minor` or `bump patch`.

> The following content is a sample. Please modify it as necessary.
### Example Change1 🧨

API::foo() was previously used but has been deprecated.
API::bar() is recommended as its successor.

before
```C++
API::foo()
```

after
```C++
API::bar()
```

## Link to the issue

> If a corresponding Issue exists, please delete this sentence and describe its number.

> If it does not exist, delete the "Link to the issue" section.

## Checklist for reviewers ☑️

All references to "You" in the following text refer to the code reviewer.

- [ ] Is this pull request written in a way that is easy to read from a third-party perspective?
- [ ] Is there sufficient information (background, purpose, specification, algorithm description, list of disruptive changes, and their migration guide) in the description of this pull request?
- [ ] If this pull request contains a destructive change, does this pull request contain the migration guide?
- [ ] Labels of this pull request are valid?
- [ ] All unit tests / integration tests are included in this pull-request? If the you think adding test case is unnecessary, please describe why and cross out this line.
- [ ] The documentation is enough? If the you think the adding documents for this pull-request, is unnecessary, please describe why please describe why and cross out this line.
