document.addEventListener('DOMContentLoaded', () => {
    const passwordProtection = document.getElementById('password-protection');
    const secretContent = document.getElementById('secret-content');
    const passwordInput = document.getElementById('password-input');
    const submitButton = document.getElementById('submit-password');
    const errorMessage = document.getElementById('error-message');

    // dang you really wanna see the secret huh?
    const correctPasswordHash = '2a0e27c0b89e3a1f1115321f513d6118025261d7986c58467433c57706a5249c'; // SHA-256 hash for "shak123"

    submitButton.addEventListener('click', () => {
        const enteredPassword = passwordInput.value;
        sha256(enteredPassword).then(hash => {
            if (hash === correctPasswordHash) {
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

    async function sha256(message) {
        const msgBuffer = new TextEncoder().encode(message);
        const hashBuffer = await crypto.subtle.digest('SHA-256', msgBuffer);
        const hashArray = Array.from(new Uint8Array(hashBuffer));
        const hashHex = hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
        return hashHex;
    }
});
