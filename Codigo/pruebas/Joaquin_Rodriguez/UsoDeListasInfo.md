# USO DE LISTAS

Las listas son un tipo de estructura de datos, son arrays. Se definen con corchetes y cada elemento de la lista es accesible e identificable por un índice.


- Es posible acceder a un elemento de lista mediante un índice que indica su posición en la misma. Debemos poner entre corchetes el número que le corresponde
al elemento que necesitamos según su posición en la lista. Pueden accederse a los últimos elementos empleando números negativos:

    lista = [a,b,c,d,e,f]
    print(lista[2]) 

Esto extraerá la letra C, ya que se empieza a contar desde el 0
A ocupa la posición 0
B ocupa la posición 1
C ocupa la posición 2
D ocupa la posición 3
E ocupa la posición 4
F ocupa la posición 5


- También pueden accederse elementos de la lista de la siguiente manera:
 lista = [a,b,c,d,e,f]
 print(lista[2:4]) Imprime: C, D, E
 print(lista[:2]) Imprime: A, B


# COMANDOS DE LISTAS

- append("elemento"): Añade un elemento al final de la lista
- insert(número de posición, "elemento"): El primer argumento, el número, indica la posición que ocupará el elemento en la lista y el elemento es lo que 
  se insertará.
- extend([lista]): Concatena listas. Inserta la lista que le ingresemos por el comando al final de la lista a la que se lo estamos agregando
- index("nombre del elemento que buscamos"): Sirve como herramienta de búsqueda. Si ingresamos el elemento que buscamos, nos devuelve la posición que ocupa
   dicho elemento en la lista
- remove("elemento"): Elimina el elemento que le pedimos
- len((lista)): Indica la longitud de una lista.

# EXTRA

- Se puede crear una lista vacía (es decir sin elementos al momento de su creación) con una cantidad de posiciones determinada:
    lista = [None]*10