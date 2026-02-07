document.addEventListener('DOMContentLoaded', () => {
    const passwordProtection = document.getElementById('password-protection');
    const secretContent = document.getElementById('secret-content');
    const passwordInput = document.getElementById('password-input');
    const submitButton = document.getElementById('submit-password');
    const errorMessage = document.getElementById('error-message');

    // The password is now checked directly.
    const correctPassword = 'shak123';

    submitButton.addEventListener('click', () => {
        const enteredPassword = passwordInput.value;
        if (enteredPassword === correctPassword) {
            const h1 = passwordProtection.querySelector('h1');
            h1.textContent = 'You are worthy to proceed.';
            passwordInput.style.display = 'none';
            submitButton.style.display = 'none';
            errorMessage.style.display = 'none';

            setTimeout(() => {
                passwordProtection.style.display = 'none';
                secretContent.style.display = 'block';
            }, 2000); // 2-second delay before showing content
        } else {
            errorMessage.textContent = 'Incorrect password.';
        }
    });
});
