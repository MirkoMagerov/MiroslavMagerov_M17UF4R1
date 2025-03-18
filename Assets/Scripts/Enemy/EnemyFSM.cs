using UnityEngine;
using UnityEngine.AI;

public class EnemyFSM : MonoBehaviour
{
    public enum EnemyState { Patrol, Chase, Attack }
    private EnemyState currentState;
    private EnemyState previousState;
    private Transform player;

    // Puntos de patrulla
    public Transform[] patrolPoints;
    private int currentPatrolIndex;
    private Vector3[] randomPatrolPoints = new Vector3[2];
    private int currentRandomPatrolIndex = 0;
    private bool usingRandomPatrolPoints = false;

    // Configuraci�n del enemigo
    public float chaseRange = 10f;
    public float attackRange = 2f;
    public float sightRange = 12f;
    public float sightAngle = 90f;
    public LayerMask obstacleLayers;

    // Configuraci�n de puntos aleatorios
    public float maxRandomPatrolDistance = 12f;
    public int maxRandomPatrolAttempts = 10;
    public float minWallDistance = 2f;
    public int wallCheckRays = 8;

    private NavMeshAgent agent;
    private Vector3 lastKnownPlayerPosition;
    private bool canSeePlayer = false;
    private float lostSightTime = 0f;
    public float timeBeforeNewPatrolPoints = 5f;

    private void Start()
    {
        player = GameObject.FindGameObjectWithTag("Player").transform;
        agent = GetComponent<NavMeshAgent>();
        currentState = EnemyState.Patrol;
        previousState = EnemyState.Patrol;
        MoveToNextPatrolPoint();
    }

    private void Update()
    {
        // Verificar si puede ver al jugador
        canSeePlayer = CanSeePlayer();

        // Si ve al jugador, actualizar su �ltima posici�n conocida
        if (canSeePlayer)
        {
            lastKnownPlayerPosition = player.position;
            lostSightTime = 0f;
        }
        else if (currentState == EnemyState.Chase)
        {
            // Contabilizar el tiempo desde que perdi� de vista al jugador
            lostSightTime += Time.deltaTime;

            // Generar nuevos puntos de patrulla despu�s de cierto tiempo
            if (lostSightTime >= timeBeforeNewPatrolPoints && !usingRandomPatrolPoints)
            {
                GenerateRandomPatrolPoints();
                usingRandomPatrolPoints = true;
            }
        }

        // Manejar el comportamiento seg�n el estado actual
        switch (currentState)
        {
            case EnemyState.Patrol:
                Patrol();
                break;
            case EnemyState.Chase:
                Chase();
                break;
            case EnemyState.Attack:
                Attack();
                break;
        }

        // Actualizar el estado anterior y verificar transiciones
        previousState = currentState;
        CheckTransitions();
    }

    private bool CanSeePlayer()
    {
        if (Vector3.Distance(transform.position, player.position) > sightRange)
            return false;

        Vector3 directionToPlayer = player.position - transform.position;

        // Verificar si el jugador est� dentro del �ngulo de visi�n
        float angle = Vector3.Angle(transform.forward, directionToPlayer);
        if (angle > sightAngle / 2)
            return false;

        // Verificar si hay obst�culos entre el enemigo y el jugador
        RaycastHit hit;
        if (Physics.Raycast(transform.position, directionToPlayer, out hit, sightRange, obstacleLayers))
        {
            if (hit.transform != player)
                return false;
        }

        return true;
    }

    private void Patrol()
    {
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            if (usingRandomPatrolPoints)
            {
                currentRandomPatrolIndex = (currentRandomPatrolIndex + 1) % randomPatrolPoints.Length;
                agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
            }
            else
            {
                MoveToNextPatrolPoint();
            }
        }
    }

    private void MoveToNextPatrolPoint()
    {
        if (patrolPoints.Length == 0) return;

        agent.SetDestination(patrolPoints[currentPatrolIndex].position);
        currentPatrolIndex = (currentPatrolIndex + 1) % patrolPoints.Length;
    }

    private void Chase()
    {
        // Si puede ver al jugador, perseguirlo directamente
        if (canSeePlayer)
        {
            agent.SetDestination(player.position);
        }
        // Si no puede verlo pero est� en modo persecuci�n, ir a su �ltima posici�n conocida
        else if (!agent.pathPending && agent.remainingDistance < 0.5f && !usingRandomPatrolPoints)
        {
            // Se ha llegado a la �ltima posici�n conocida del jugador
            // Esperar a que se cumplan los segundos antes de generar puntos aleatorios
            // (El tiempo se cuenta en Update)
        }
    }

    private void Attack()
    {
        agent.ResetPath(); // Se detiene
        // Aqu� puedes agregar la l�gica de ataque
        Debug.Log("Atacando al jugador!");
    }

    private void GenerateRandomPatrolPoints()
    {
        for (int i = 0; i < randomPatrolPoints.Length; i++)
        {
            Vector3 randomPoint = GetRandomPointOnNavMesh(lastKnownPlayerPosition, maxRandomPatrolDistance);
            randomPatrolPoints[i] = randomPoint;
        }

        currentRandomPatrolIndex = 0;
        agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
    }

    private Vector3 GetRandomPointOnNavMesh(Vector3 center, float maxDistance)
    {
        for (int i = 0; i < maxRandomPatrolAttempts; i++)
        {
            // Generar un punto aleatorio en un c�rculo alrededor del centro
            Vector2 randomPoint2D = Random.insideUnitCircle * maxDistance;
            Vector3 randomPoint = center + new Vector3(randomPoint2D.x, 0, randomPoint2D.y);

            // Verificar si el punto est� en el NavMesh
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, maxDistance, NavMesh.AllAreas))
            {
                // Comprobar que est� lejos de paredes
                if (IsPointAwayFromWalls(hit.position))
                {
                    // Verificar l�nea de visi�n desde el punto al centro
                    Vector3 directionToCenter = center - hit.position;
                    RaycastHit obstacleHit;
                    if (!Physics.Raycast(hit.position, directionToCenter, out obstacleHit, maxDistance, obstacleLayers))
                    {
                        return hit.position;
                    }
                }
            }
        }

        // Si no se encuentra un punto v�lido, intentar ajustar el centro
        NavMeshHit centerHit;
        if (NavMesh.SamplePosition(center, out centerHit, maxDistance, NavMesh.AllAreas))
        {
            // Intentar alejar el punto del centro de paredes cercanas
            Vector3 safePoint = AdjustPointAwayFromWalls(centerHit.position);
            return safePoint;
        }

        // Si todo falla, retornar el centro
        return center;
    }

    // Verifica si un punto est� a una distancia segura de las paredes
    private bool IsPointAwayFromWalls(Vector3 point)
    {
        // Lanzar rayos en varias direcciones para verificar paredes cercanas
        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            // Si hay una pared a menos de la distancia m�nima, rechazar el punto
            if (Physics.Raycast(point, direction, out hit, minWallDistance, obstacleLayers))
            {
                return false;
            }
        }
        return true;
    }

    // Intenta ajustar un punto alej�ndolo de paredes cercanas
    private Vector3 AdjustPointAwayFromWalls(Vector3 point)
    {
        Vector3 adjustedPoint = point;
        Vector3 pushDirection = Vector3.zero;
        bool wallDetected = false;

        // Detectar paredes cercanas y calcular direcci�n de "empuje"
        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(point, direction, out hit, minWallDistance * 1.5f, obstacleLayers))
            {
                // Calcular vector de "empuje" alej�ndose de la pared
                float pushStrength = minWallDistance - hit.distance;
                pushDirection += -direction.normalized * pushStrength;
                wallDetected = true;
            }
        }

        // Si se detectaron paredes, mover el punto en la direcci�n opuesta
        if (wallDetected)
        {
            adjustedPoint += pushDirection.normalized * minWallDistance;

            // Asegurarse de que el punto ajustado est� en el NavMesh
            NavMeshHit navHit;
            if (NavMesh.SamplePosition(adjustedPoint, out navHit, minWallDistance, NavMesh.AllAreas))
            {
                return navHit.position;
            }
        }

        return adjustedPoint;
    }

    private void CheckTransitions()
    {
        float distanceToPlayer = Vector3.Distance(transform.position, player.position);

        // Transiciones de estado
        if (distanceToPlayer <= attackRange && canSeePlayer)
        {
            currentState = EnemyState.Attack;
        }
        else if (distanceToPlayer <= chaseRange && canSeePlayer)
        {
            currentState = EnemyState.Chase;
            usingRandomPatrolPoints = false; // Volver a perseguir directamente
        }
        else if (currentState == EnemyState.Chase &&
                 lostSightTime >= timeBeforeNewPatrolPoints &&
                 !canSeePlayer &&
                 !agent.pathPending &&
                 agent.remainingDistance < 0.5f)
        {
            // Transici�n a patrulla cuando lleva mucho tiempo sin ver al jugador y ya lleg� a su �ltima posici�n conocida
            currentState = EnemyState.Patrol;
        }
        else if (currentState != EnemyState.Chase && canSeePlayer && distanceToPlayer <= chaseRange)
        {
            // Transici�n a persecuci�n si ve al jugador y est� dentro del rango
            currentState = EnemyState.Chase;
            usingRandomPatrolPoints = false;
        }
    }

    // Visualizaci�n de gizmos en el editor
    private void OnDrawGizmos()
    {
        // Dibuja el rango de persecuci�n
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, chaseRange);

        // Dibuja el rango de ataque
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, attackRange);

        // Dibuja el rango de visi�n como un cono
        Gizmos.color = canSeePlayer ? Color.green : Color.blue;
        DrawVisionCone();

        // Dibuja l�nea de visi�n hacia el jugador si est� en la escena
        if (player != null)
        {
            Vector3 directionToPlayer = player.position - transform.position;
            float distanceToPlayer = directionToPlayer.magnitude;

            // Si puede ver al jugador
            if (canSeePlayer)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position, player.position);
            }
            // Si est� dentro del rango pero no puede verlo (obst�culos)
            else if (distanceToPlayer <= sightRange)
            {
                Gizmos.color = Color.red;
                RaycastHit hit;
                if (Physics.Raycast(transform.position, directionToPlayer.normalized, out hit, sightRange, obstacleLayers))
                {
                    Gizmos.DrawLine(transform.position, hit.point);
                    // Dibujar una X en el punto de impacto
                    float crossSize = 0.3f;
                    Gizmos.DrawLine(hit.point + new Vector3(crossSize, crossSize, crossSize),
                                    hit.point + new Vector3(-crossSize, -crossSize, -crossSize));
                    Gizmos.DrawLine(hit.point + new Vector3(-crossSize, crossSize, crossSize),
                                    hit.point + new Vector3(crossSize, -crossSize, -crossSize));
                }
            }
        }

        // Dibuja los puntos de patrulla aleatorios si est�n en uso
        if (Application.isPlaying && usingRandomPatrolPoints)
        {
            // Dibuja los puntos aleatorios de patrulla
            Gizmos.color = Color.magenta;
            foreach (Vector3 point in randomPatrolPoints)
            {
                Gizmos.DrawSphere(point, 0.3f);

                // Dibuja los rayos de comprobaci�n de paredes para cada punto
                Gizmos.color = Color.cyan;
                for (int i = 0; i < wallCheckRays; i++)
                {
                    float angle = i * (360f / wallCheckRays);
                    Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    Gizmos.DrawRay(point, direction * minWallDistance);
                }
            }

            // Dibuja la �ltima posici�n conocida del jugador
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(lastKnownPlayerPosition, 0.5f);
            Gizmos.DrawLine(lastKnownPlayerPosition + Vector3.up * 0.5f,
                            lastKnownPlayerPosition + Vector3.up * 2f);
        }
    }

    // M�todo para dibujar un cono de visi�n
    private void DrawVisionCone()
    {
        float halfAngle = sightAngle / 2;
        Vector3 forward = transform.forward;
        Vector3 right = transform.right;

        // N�mero de l�neas para dibujar el cono
        int lineCount = 20;

        // Dibujar l�neas para representar el cono
        for (int i = 0; i <= lineCount; i++)
        {
            float angle = -halfAngle + (sightAngle * i / lineCount);
            float radians = angle * Mathf.Deg2Rad;

            // Calcular direcci�n usando rotaci�n
            Vector3 direction = forward * Mathf.Cos(radians) + right * Mathf.Sin(radians);
            direction.Normalize();

            // Dibujar rayo
            Gizmos.DrawRay(transform.position, direction * sightRange);
        }

        // Dibujar arcos para representar los "anillos" del cono
        int arcCount = 5;
        for (int i = 1; i <= arcCount; i++)
        {
            float radius = sightRange * i / arcCount;
            Vector3 lastPoint = Vector3.zero;

            for (int j = 0; j <= lineCount; j++)
            {
                float angle = -halfAngle + (sightAngle * j / lineCount);
                float radians = angle * Mathf.Deg2Rad;

                Vector3 direction = forward * Mathf.Cos(radians) + right * Mathf.Sin(radians);
                direction.Normalize();

                Vector3 currentPoint = transform.position + direction * radius;

                if (j > 0)
                {
                    Gizmos.DrawLine(lastPoint, currentPoint);
                }

                lastPoint = currentPoint;
            }
        }
    }
}