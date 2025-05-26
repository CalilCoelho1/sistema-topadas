git config --global user.name "CalilCoelho1"
git config --global user.email "calilcoelho1@gmail.com"

# Ver status dos arquivos
git status

# Adicionar mudanças
git add .                    # Todos os arquivos
git add src/main.cpp         # Arquivo específico

# Fazer commit
git commit -m "📝 Descrição da mudança"

# Enviar para GitHub
git push

# Baixar atualizações
git pull

///////////////////////////////////

# Versões e Tags

bash# Criar tag de versão
git tag -a v1.0.0 -m "Versão 1.0.0 - Lançamento inicial"

# Enviar tag
git push origin v1.0.0

# Listar tags
git tag

# Ver informações da tag
git show v1.0.0