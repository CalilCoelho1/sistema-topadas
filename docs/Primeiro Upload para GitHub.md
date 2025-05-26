git config --global user.name "CalilCoelho1"
git config --global user.email "calilcoelho1@gmail.com"

# 1. Inicializar repositório
git init

# 2. Adicionar arquivos
git add .

# 3. Primeiro commit
git commit -m "🎉 Initial commit"

# 4. Conectar ao GitHub
git remote add origin https://github.com/CalilCoelho1/NOME-REPOSITORIO.git

# 5. Configurar branch main
git branch -M main

# 6. Enviar para GitHub
git push -u origin main