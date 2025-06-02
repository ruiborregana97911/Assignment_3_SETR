# README - Projeto Unity

Este projeto contém uma estrutura básica para testes unitários usando o framework Unity, adaptada para o seu ficheiro main.c.

## Estrutura do Projeto

```
novo_projeto/
├── CMakeLists.txt                  # Ficheiro de configuração CMake principal
├── src/
│   ├── CMakeLists.txt              # Configuração CMake para os ficheiros fonte
│   ├── main.c                      # O seu código principal
│   ├── unity.c                     # Implementação do Unity
│   ├── unity.h                     # Header principal do Unity
│   └── unity_internals.h           # Headers internos do Unity
└── test/
    ├── test_main.c                 # Testes para o main.c
    └── test_runners/
        └── test_main_runner.c      # Runner para executar os testes
```

## Como Usar

### Compilação do Projeto

Para compilar o projeto, siga estes passos:

1. Crie um diretório de build:
   ```
   mkdir build && cd build
   ```

2. Configure o projeto com CMake:
   ```
   cmake ..
   ```

3. Compile o projeto:
   ```
   make
   ```

### Execução dos Testes

Para executar os testes:

```
cd build
ctest
```

Ou execute diretamente o runner de testes:

```
./test_main_runner
```

## Adaptação para o Seu Ambiente

Este projeto foi configurado para funcionar com o seu ficheiro main.c, mas pode ser necessário fazer algumas adaptações:

1. Os mocks em `test_main.c` podem precisar de ser ajustados conforme as dependências específicas do seu ambiente.
2. Se estiver a usar o Zephyr RTOS, pode ser necessário ajustar a configuração do CMake para incluir as bibliotecas do Zephyr.
3. Adicione mais testes conforme necessário para cobrir outras funções do seu código.

## Estrutura de Testes

Os testes foram organizados seguindo as melhores práticas do Unity:

1. Cada função de teste começa com `test_` seguido do nome da função que está a ser testada.
2. Os mocks são usados para simular o comportamento de dependências externas.
3. O runner de testes executa automaticamente todos os testes definidos.

## Personalização

Para adicionar mais testes:

1. Adicione novas funções de teste em `test_main.c`.
2. Atualize o runner em `test_runners/test_main_runner.c` para incluir os novos testes.

## Referências

- [Documentação do Unity](https://github.com/ThrowTheSwitch/Unity)
- [Guia de Testes Unitários para Embedded C](http://www.throwtheswitch.org/unity)
