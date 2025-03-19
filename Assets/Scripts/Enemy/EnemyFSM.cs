using System.Collections;
using UnityEngine;
using UnityEngine.AI;

public class EnemyFSM : MonoBehaviour
{
    public enum EnemyState { Patrol, Chase, Attack, Flee }
    private EnemyState currentState;
    private EnemyState previousState;
    private Transform player;
    private EnemyLife enemyLife;

    // Puntos de patrulla
    public Transform[] patrolPoints;
    private int currentPatrolIndex;
    private Vector3[] randomPatrolPoints = new Vector3[2];
    private int currentRandomPatrolIndex = 0;
    private bool usingRandomPatrolPoints = false;

    // Configuración del enemigo
    public float chaseRange = 10f;
    public float attackRange = 2f;
    public float sightRange = 12f;
    public float sightAngle = 90f;
    public LayerMask obstacleLayers;

    // Configuración de puntos aleatorios
    public float maxRandomPatrolDistance = 12f;
    public int maxRandomPatrolAttempts = 10;
    public float minWallDistance = 2f;
    public int wallCheckRays = 8;
    public float minDistanceBetweenPatrolPoints = 5f;
    public bool validateNavMeshPaths = true;

    // Configuración de huida
    public float fleeDistance = 15f;
    public float fleeUpdateInterval = 1f;
    private float fleeTimer = 0f;

    private NavMeshAgent agent;
    private Vector3 lastKnownPlayerPosition;
    private bool canSeePlayer = false;
    private float lostSightTime = 0f;
    public float timeBeforeNewPatrolPoints = 5f;
    private float originalSpeed;

    private void Start()
    {
        enemyLife = GetComponent<EnemyLife>();
        player = GameObject.FindGameObjectWithTag("Player").transform;
        agent = GetComponent<NavMeshAgent>();
        currentState = EnemyState.Patrol;
        previousState = EnemyState.Patrol;
        originalSpeed = agent.speed;

        // Validar puntos de patrulla existentes
        if (validateNavMeshPaths && patrolPoints.Length > 0)
        {
            ValidatePatrolPoints();
        }

        MoveToNextPatrolPoint();
    }

    private void Update()
    {
        // Verificar si puede ver al jugador
        canSeePlayer = CanSeePlayer();

        // Si ve al jugador, actualizar su última posición conocida
        if (canSeePlayer)
        {
            lastKnownPlayerPosition = player.position;
            lostSightTime = 0f;
        }
        else if (currentState == EnemyState.Chase)
        {
            // Contabilizar el tiempo desde que perdió de vista al jugador
            lostSightTime += Time.deltaTime;

            // Generar nuevos puntos de patrulla después de cierto tiempo
            if (lostSightTime >= timeBeforeNewPatrolPoints && !usingRandomPatrolPoints)
            {
                GenerateRandomPatrolPoints();
                usingRandomPatrolPoints = true;
            }
        }

        // Manejar el comportamiento según el estado actual
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
            case EnemyState.Flee:
                Flee();
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

        // Verificar si el jugador está dentro del ángulo de visión
        float angle = Vector3.Angle(transform.forward, directionToPlayer);
        if (angle > sightAngle / 2)
            return false;

        // Verificar si hay obstáculos entre el enemigo y el jugador
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
    }

    private void Attack()
    {
        agent.speed = 0;
        Debug.Log("Atacando al jugador!");
        StartCoroutine(WaitForSeconds(1f));
        agent.speed = originalSpeed;
    }

    private IEnumerator WaitForSeconds(float seconds)
    {
        yield return new WaitForSeconds(seconds);
    }

    private void Flee()
    {
        // Actualizar la dirección de huida periódicamente
        fleeTimer += Time.deltaTime;
        if (fleeTimer >= fleeUpdateInterval || !agent.hasPath)
        {
            fleeTimer = 0f;

            if (canSeePlayer)
            {
                // Calcular dirección opuesta al jugador
                Vector3 fleeDirection = transform.position - player.position;
                fleeDirection.y = 0; // Mantener en el mismo plano

                // Buscar un punto de huida
                Vector3 targetPosition = transform.position + fleeDirection.normalized * fleeDistance;

                // Ajustar al NavMesh
                NavMeshHit hit;
                if (NavMesh.SamplePosition(targetPosition, out hit, fleeDistance, NavMesh.AllAreas))
                {
                    agent.SetDestination(hit.position);
                }
            }
            else
            {
                // Si no ve al jugador, buscar un punto alejado en dirección aleatoria
                Vector3 randomDirection = Random.insideUnitSphere * fleeDistance;
                randomDirection.y = 0;
                Vector3 targetPosition = transform.position + randomDirection;

                NavMeshHit hit;
                if (NavMesh.SamplePosition(targetPosition, out hit, fleeDistance, NavMesh.AllAreas))
                {
                    agent.SetDestination(hit.position);
                }
            }
        }
    }

    // Verificar que todos los puntos de patrulla tengan rutas válidas entre ellos
    private void ValidatePatrolPoints()
    {
        for (int i = 0; i < patrolPoints.Length; i++)
        {
            if (patrolPoints[i] == null) continue;

            // Verificar ruta desde este punto al siguiente
            int nextIndex = (i + 1) % patrolPoints.Length;
            NavMeshPath path = new NavMeshPath();

            bool validPath = NavMesh.CalculatePath(
                patrolPoints[i].position,
                patrolPoints[nextIndex].position,
                NavMesh.AllAreas,
                path
            );

            if (!validPath || path.status != NavMeshPathStatus.PathComplete)
            {
                Debug.LogWarning($"No valid NavMesh path between patrol points {i} and {nextIndex}!");
            }
        }
    }

    private void GenerateRandomPatrolPoints()
    {
        int attemptCount = 0;
        int maxAttempts = maxRandomPatrolAttempts * 3;

        // Generar el primer punto
        randomPatrolPoints[0] = GetRandomPointOnNavMesh(lastKnownPlayerPosition, maxRandomPatrolDistance);

        // Generar puntos restantes con distancia mínima
        for (int i = 1; i < randomPatrolPoints.Length; i++)
        {
            Vector3 validPoint = Vector3.zero;
            bool foundValidPoint = false;

            while (!foundValidPoint && attemptCount < maxAttempts)
            {
                attemptCount++;
                Vector3 candidatePoint = GetRandomPointOnNavMesh(lastKnownPlayerPosition, maxRandomPatrolDistance);

                // Verificar distancia mínima entre puntos
                bool isFarEnough = true;
                for (int j = 0; j < i; j++)
                {
                    if (Vector3.Distance(candidatePoint, randomPatrolPoints[j]) < minDistanceBetweenPatrolPoints)
                    {
                        isFarEnough = false;
                        break;
                    }
                }

                // Verificar si hay ruta de NavMesh válida desde el punto anterior
                if (isFarEnough && validateNavMeshPaths)
                {
                    NavMeshPath path = new NavMeshPath();
                    if (!NavMesh.CalculatePath(randomPatrolPoints[i - 1], candidatePoint, NavMesh.AllAreas, path) ||
                        path.status != NavMeshPathStatus.PathComplete)
                    {
                        isFarEnough = false;
                    }
                }

                if (isFarEnough)
                {
                    validPoint = candidatePoint;
                    foundValidPoint = true;
                }
            }

            // Si no se encuentra punto válido, crear uno en otra dirección
            if (!foundValidPoint)
            {
                // Intentar varias direcciones hasta encontrar una con camino válido
                for (int attempt = 0; attempt < 8; attempt++)
                {
                    float angle = attempt * (360f / 8);
                    Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    Vector3 targetPosition = lastKnownPlayerPosition + direction * minDistanceBetweenPatrolPoints * 1.5f;

                    NavMeshHit hit;
                    if (NavMesh.SamplePosition(targetPosition, out hit, maxRandomPatrolDistance, NavMesh.AllAreas))
                    {
                        NavMeshPath path = new NavMeshPath();
                        if (!validateNavMeshPaths ||
                            (NavMesh.CalculatePath(randomPatrolPoints[i - 1], hit.position, NavMesh.AllAreas, path) &&
                             path.status == NavMeshPathStatus.PathComplete))
                        {
                            validPoint = hit.position;
                            foundValidPoint = true;
                            break;
                        }
                    }
                }

                // Si aún no hay punto válido, usar el mismo que el anterior
                if (!foundValidPoint)
                {
                    validPoint = randomPatrolPoints[i - 1];
                }
            }

            randomPatrolPoints[i] = validPoint;
        }

        currentRandomPatrolIndex = 0;
        agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
    }

    private Vector3 GetRandomPointOnNavMesh(Vector3 center, float maxDistance)
    {
        for (int i = 0; i < maxRandomPatrolAttempts; i++)
        {
            // Generar un punto aleatorio en un círculo alrededor del centro
            Vector2 randomPoint2D = Random.insideUnitCircle * maxDistance;
            Vector3 randomPoint = center + new Vector3(randomPoint2D.x, 0, randomPoint2D.y);

            // Verificar si el punto está en el NavMesh
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, maxDistance, NavMesh.AllAreas))
            {
                // Comprobar que esté lejos de paredes
                if (IsPointAwayFromWalls(hit.position))
                {
                    // Verificar línea de visión desde el punto al centro
                    Vector3 directionToCenter = center - hit.position;
                    RaycastHit obstacleHit;
                    if (!Physics.Raycast(hit.position, directionToCenter, out obstacleHit, maxDistance, obstacleLayers))
                    {
                        return hit.position;
                    }
                }
            }
        }

        // Si no se encuentra un punto válido, intentar ajustar el centro
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

    // Verifica si un punto está a una distancia segura de las paredes
    private bool IsPointAwayFromWalls(Vector3 point)
    {
        // Lanzar rayos en varias direcciones para verificar paredes cercanas
        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            // Si hay una pared a menos de la distancia mínima, rechazar el punto
            if (Physics.Raycast(point, direction, out hit, minWallDistance, obstacleLayers))
            {
                return false;
            }
        }
        return true;
    }

    // Intenta ajustar un punto alejándolo de paredes cercanas
    private Vector3 AdjustPointAwayFromWalls(Vector3 point)
    {
        Vector3 adjustedPoint = point;
        Vector3 pushDirection = Vector3.zero;
        bool wallDetected = false;

        // Detectar paredes cercanas y calcular dirección de "empuje"
        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(point, direction, out hit, minWallDistance * 1.5f, obstacleLayers))
            {
                // Calcular vector de "empuje" alejándose de la pared
                float pushStrength = minWallDistance - hit.distance;
                pushDirection += -direction.normalized * pushStrength;
                wallDetected = true;
            }
        }

        // Si se detectaron paredes, mover el punto en la dirección opuesta
        if (wallDetected)
        {
            adjustedPoint += pushDirection.normalized * minWallDistance;

            // Asegurarse de que el punto ajustado esté en el NavMesh
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
        bool isHealthLow = enemyLife.IsLowHealth();

        // Priorizar huida si la salud es baja y ve al jugador
        if (isHealthLow && canSeePlayer && distanceToPlayer <= chaseRange)
        {
            currentState = EnemyState.Flee;
        }
        // Estado de ataque si está cerca y tiene salud suficiente
        else if (distanceToPlayer <= attackRange && canSeePlayer && !isHealthLow)
        {
            currentState = EnemyState.Attack;
        }
        // Estado de persecución si está en rango y tiene salud suficiente
        else if (distanceToPlayer <= chaseRange && canSeePlayer && !isHealthLow)
        {
            currentState = EnemyState.Chase;
            usingRandomPatrolPoints = false;
        }
        // Volver a patrulla si ha perdido de vista al jugador por mucho tiempo
        else if (currentState == EnemyState.Chase &&
                 lostSightTime >= timeBeforeNewPatrolPoints &&
                 !canSeePlayer &&
                 !agent.pathPending &&
                 agent.remainingDistance < 0.5f)
        {
            currentState = EnemyState.Patrol;
        }
        // Volver a patrulla si estaba huyendo y ya no ve al jugador
        else if (currentState == EnemyState.Flee &&
                 (!canSeePlayer || distanceToPlayer > chaseRange * 1.5f))
        {
            currentState = EnemyState.Patrol;
        }
    }

    // Visualización de gizmos en el editor
    private void OnDrawGizmos()
    {
        // Dibuja el rango de persecución
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, chaseRange);

        // Dibuja el rango de ataque
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, attackRange);

        // Dibuja el rango de visión como un cono
        Gizmos.color = canSeePlayer ? Color.green : Color.blue;
        DrawVisionCone();
    }

    // Método para dibujar un cono de visión
    private void DrawVisionCone()
    {
        float halfAngle = sightAngle / 2;
        Vector3 forward = transform.forward;
        Vector3 right = transform.right;

        // Número de líneas para dibujar el cono
        int lineCount = 10;

        // Dibujar líneas para representar el cono
        for (int i = 0; i <= lineCount; i++)
        {
            float angle = -halfAngle + (sightAngle * i / lineCount);
            float radians = angle * Mathf.Deg2Rad;

            // Calcular dirección usando rotación
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