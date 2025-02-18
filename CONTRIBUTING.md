# Contributing

Thank you for your interest in contributing to this library! Please follow the guidelines below to ensure an organized and consistent workflow.

## Fork and Pull Request Workflow

We use **GitFlow** to manage project development. This means that:

- Contributors must **fork** the repository before making changes.
- All changes should be made in a separate branch within your fork.
- Once the changes are complete, submit a **pull request** to merge into the `dev` branch of the main repository.
- Periodically, stable changes from `dev` are merged into `main`.

## Contribution Guidelines

1. **Fork the Repository**: Click the `Fork` button on GitHub and clone your fork locally:
   ```sh
   git clone https://github.com/your-username/project-name.git
   cd project-name
   ```

2. **Create a Branch**: Always create a new branch for your changes:
   ```sh
   git checkout -b feature/feature-name
   ```

3. **Implement Changes**: Write clean, well-documented code and ensure it follows project standards.

4. **Commit Your Changes**: Use clear and descriptive commit messages:
   ```sh
   git commit -m "Added feature X to handle Y"
   ```

5. **Push to Your Fork**: Push your changes to your forked repository:
   ```sh
   git push origin feature/feature-name
   ```

6. **Open a Pull Request**: Go to the main repository on GitHub and open a pull request from your branch to the `dev` branch.

## Additional Rules

1. The `main` branch should only contain stable and release-ready code.
2. Avoid working directly on `main`.
3. Always sync your fork with the latest changes from `dev` before starting new work:
   ```sh
   git checkout dev
   git pull upstream dev
   ```
4. Your pull request may require approval and code review before being merged.

Thank you for your contributions!
