# FRC_Programming_2026

## Setup

```sh
git clone https://github.com/Team3256/FRC_Programming_2025.git
# Only needs to run this once to install deps
./gradlew build
```

Optionally, install the pre-commit hook:

```shell
cat <<EOF > .git/hooks/pre-commit
#!/bin/sh
stagedFiles=$(git diff --staged --name-only)
echo "Running spotlessApply. Formatting code..."
./gradlew spotlessApply
for file in $stagedFiles; do
  if test -f "$file"; then
    git add $file
  fi
done
EOF
```

## Development

Run Glass sim:

```sh
./gradlew simulateJava
```

Or to deploy to the real robot, make sure you have the ethernet connected to your computer and then run:

```
./gradlew deploy
```

Before you commit your changes, remember to run Spotless:

```sh
./gradlew spotlessApply
```
